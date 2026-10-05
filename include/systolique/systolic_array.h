// SystolicArray: Gemmini's systolic array (MeshWithDelays with its Mesh of Tiles of PEs, Gemmini
// v0.7.2) as one C++ object that is explicit about cycles, occupancy and the provenance of every
// value the PEs hold. Docs: docs/systolic_array.md.
//
// The array itself is the component classes (MeshWithDelays owning Mesh, Transposer, queues;
// Mesh owning Tiles owning PEs), not a second implementation. Everything this class adds only
// observes their state: which row of which request each PE works on in each cycle, what that
// PE-cycle is (the PeState partition), and where each register value came from. The provenance
// comes from a second MeshWithDelays ("token twin") that receives the same requests and
// handshakes but a unique token instead of every d value, so a token moves through the
// transposer, skew registers, inter-tile pipes and PE registers exactly as the value it stands
// for (systolic_array.cpp, Twin).
//
// Cycles. cycle() counts clock cycles from reset release: cycle 0 is the first cycle with reset
// low after the last cycle with reset high (or the first cycle after construction). "In cycle t"
// means between the rising edges that start and end cycle t: the inputs applied in t, the
// outputs seen in t (they depend on registers only), the registers' values in t; a PE that is
// valid in t updates its registers at the edge that ends t (PE.scala:141-146).
//
// Clock. out() (mesh_out() on the bare Mesh interface) gives this cycle's outputs before any
// input is chosen; set_inputs() gives this cycle's inputs; tick() takes the edge that ends the
// cycle: every component evaluates from the pre-edge registers, the accounting records the
// cycle, then every component commits (two-phase, see mesh_with_delays.h).
//
// One PE = one MAC per cycle. A PE has one MacUnit (PE.scala:64-65) and uses it at most once per
// cycle, so every PE-cycle is one MAC slot and the array's peak is rows() x cols() MACs per
// cycle, however the PEs are grouped into tiles. Each PE-cycle gets exactly one state of PeState
// (a partition), and a PE-cycle counts as a useful MAC iff its state is Mac: useful MACs of a run
// = PE-cycles in Mac. Work the PE does in the same cycle besides that MAC (a WS weight or OS D
// shifting into the other register, an OS result shifting out) is recorded as a non-exclusive
// flag (load_concurrent, drain_concurrent), never as a second state or a second MAC.
#pragma once

#include "systolique/config.h"
#include "systolique/mesh.h"
#include "systolique/mesh_with_delays.h"
#include "systolique/types.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace systolique {

// What a PE does with its one MAC slot in one cycle; a closed set, exactly one per PE-cycle.
// The rule (first match wins), for PE (r, c) in cycle t, with the row it holds attributed to its
// request q (the m-th valid cycle of every PE processes the m-th row fed into the Mesh):
//   Idle    in_valid low: the registers hold (PE.scala:141-146), the MacUnit result is unused.
//   Drain   q is a flush (req.flush > 0) in OS: out_c shifts the finished results down
//           (PE.scala:104, 111) while the accumulator multiplies the stale buffers.
//   Bubble  q is a flush in WS: valid rows that carry nothing (a/b/d are stale buffers,
//           MeshWithDelays.scala:110).
//   Mac     q computes (it carries an A operand): the MacUnit's product is part of the
//           workload, WS out_b = b + a*w (PE.scala:119-130), OS c += a*b (PE.scala:107, 114).
//   Load    q does not compute but preloads (it carries a D operand): only the shadow register
//           takes in_d, WS weights (PE.scala:124, 130), OS D (PE.scala:109, 116).
//   Bubble  anything else (valid, neither A nor D).
// "Computes" and "preloads" come from a RequestNote if the caller gave one, else from the data:
// a request computes iff any A value the array received for its rows (after the transposer,
// MeshWithDelays.scala:170) is non-zero, it preloads iff any such D value is (:172). Both are
// final once the request's last row has entered the array (RequestInfo::decided).
enum class PeState : uint8_t { Idle = 0, Mac = 1, Load = 2, Drain = 3, Bubble = 4 };
constexpr unsigned kPeStates = 5;
const char *to_string(PeState s);

// PE-cycles per state, and the non-exclusive flags of the Mac PE-cycles among them.
struct StateCounts {
  std::array<uint64_t, kPeStates> n{};
  // Mac PE-cycles whose request also preloads: the shadow register takes a W (WS) / D (OS)
  // value of the operand in the same cycle (overlap of preload j with compute j-1).
  uint64_t load_concurrent = 0;
  // OS Mac PE-cycles in which out_c carries a finished result down (the register leaving through
  // out_c, PE.scala:104/111, holds a value that was accumulated; needs provenance).
  uint64_t drain_concurrent = 0;
  uint64_t operator[](PeState s) const { return n[unsigned(s)]; }
  uint64_t &operator[](PeState s) { return n[unsigned(s)]; }
  uint64_t total() const;
  uint64_t occupied() const { return total() - n[0]; }  // PE-cycles with in_valid
  uint64_t macs() const { return n[unsigned(PeState::Mac)]; }
  void add(const StateCounts &o);
};

// Optional knowledge about a request, given with the cycle in which it is offered (used if it
// fires then). -1 = decide from the data (see PeState).
struct RequestNote {
  int computes = -1, preloads = -1;
  std::string label;
};

// One request (MeshWithDelays interface) or, on the bare Mesh interface, one op: a run of valid
// rows on lane 0 with the same id, ended by a row with last.
struct RequestInfo {
  unsigned index = 0;
  MwdReq req;               // as accepted (Mesh interface: dataflow/propagate/shift/id of its first row)
  Dataflow dataflow = Dataflow::OS;  // effective at the PEs (the configuration's unless BOTH)
  std::string label;
  // Cycles, -1 if it has not happened. accept: req.valid & req.ready. first_in/last_in: its
  // first/last row at the Mesh input (lane group 0, which has no skew delay); PE (0, 0) works on
  // a row tile_latency + 1 cycles later. first_out/last_out: its own rows leaving the array
  // (resp.valid; WS: the results its rows computed, OS: whatever its rows pushed out).
  // result_first/result_last: response rows carrying its tag (TagQueue head, MeshWithDelays.scala
  // 228-235): its matmul's result (WS: computed by the next request's rows, OS: the one after).
  int64_t accept = -1, first_in = -1, last_in = -1, first_out = -1, last_out = -1;
  int64_t result_first = -1, result_last = -1;
  unsigned rows_in = 0, rows_out = 0, result_rows = 0;
  bool flush = false;
  bool computes = false, preloads = false;  // current decision (see PeState)
  bool decided = false;                     // final: note given, or the last row is in
  int note_computes = -1, note_preloads = -1;
  bool a_nonzero = false, d_nonzero = false;  // seen in its rows so far
  uint64_t valid_pe_cycles = 0;  // PE-cycles its rows occupied (all in state())
  uint64_t result_out_pe_cycles = 0;  // of those, OS cycles with a result leaving through out_c
  PeState state() const;
};

// Where a register value came from. A value enters through d: a row of the d input (one handshake,
// d.valid & d.ready) consumed into the array by the first row that enters after it.
struct RegSource {
  int request = -1;      // request that consumed the d row (-1: power-on / before reset / unknown)
  int loaded_by = -1;    // request whose row wrote it into this PE
  int64_t d_row = -1;    // number of the d handshake since reset (0 = first)
  unsigned row = 0;      // fire_counter when consumed: row of `request`'s d operand, 0 = first sent
  unsigned lane = 0;     // d lane
  bool transposed = false;  // WS bd_transpose: through the transposer (MeshWithDelays.scala:154)
  // The element of the operand matrix (W in WS, D in OS) as the ExecuteController lays it out:
  // D/W rows are sent bottom row first (ExecuteController.scala:253), so row r of the d input
  // holds matrix row DIM-1-r; through the transposer, lane l of row r holds element (l, DIM-1-r).
  int w_row = -1, w_col = -1;
  int64_t value = 0;     // the d value as received
  int64_t loaded = -1;   // cycle at whose end this PE latched it (it is in the register from loaded+1)
  // Use since loaded: WS the stationary multiplicand (PE.scala:121, 127), OS the accumulator
  // (PE.scala:107, 114). active: first cycle it was the PE's active register while valid (the
  // propagate flip that made it active); last_used, uses: last cycle / number of cycles the
  // MacUnit used it (whatever the state); last_use_request: the request of that row.
  int64_t active = -1, last_used = -1;
  uint32_t uses = 0;
  int last_use_request = -1;
};

// A d handshake row.
struct DRowInfo {
  int64_t cycle = -1;       // handshake cycle
  int request = -1;         // consumer (see RegSource), -1 if overwritten before use
  unsigned row = 0;
  bool transposed = false;
  std::vector<int64_t> values;
};

// PE (r, c) in the current cycle (before tick()), including what it does in this cycle: the
// active register's active/last_used/uses count this cycle if the PE is valid.
struct PeView {
  Mesh::PeInputs in;        // control, valid, id, last seen this cycle
  Dataflow dataflow = Dataflow::OS;  // effective
  int request = -1;         // request of the row it works on (-1: idle)
  unsigned row_in_request = 0;
  unsigned active = 0;      // 0: c1, 1: c2 -- WS the multiplicand, OS the accumulator; selected
                            // by propagate (PE.scala:103, 119: propagate = 1 -> c2), by last_s
                            // (the propagate of its last valid cycle, PE.scala:89) when idle
  PeState state = PeState::Idle;
  bool state_final = true;  // false while the request's flags may still change
  bool load_concurrent = false, drain_concurrent = false;
  // OS, valid: the register leaving through out_c holds a finished result (whatever the state;
  // drain_concurrent = state Mac and result_out). Needs provenance.
  bool result_out = false;
  PeRegs regs;
  RegSource src[2];         // of c1, c2 (empty without provenance)
};

// Counters of a run. Windows: "run" = cycles [0, cycles); "busy" = [busy_begin, busy_end], from the
// first request accepted (Mesh interface: first row in) to the last cycle with an occupied PE.
struct Accounting {
  unsigned pes = 0;          // rows() x cols(): peak MACs per cycle
  int64_t cycles = 0;
  int64_t busy_begin = -1, busy_end = -1;
  std::vector<StateCounts> per_cycle;    // [cycle], each sums to pes
  std::vector<StateCounts> per_pe;       // [row * cols + col], each sums to cycles
  std::vector<StateCounts> per_request;  // occupied PE-cycles only (no Idle)
  StateCounts total, busy;
  // occupancy = PE-cycles with in_valid / (pes x cycles of the window);
  // utilisation = useful MAC PE-cycles (state Mac) / (pes x cycles of the window).
  double occupancy = 0, utilisation = 0;            // run window
  double busy_occupancy = 0, busy_utilisation = 0;  // busy window
  int64_t busy_cycles() const { return busy_end < busy_begin ? 0 : busy_end - busy_begin + 1; }
};

enum class Interface { MeshWithDelays, Mesh };

struct ArrayOptions {
  Interface interface = Interface::MeshWithDelays;
  bool provenance = true;   // run the token twin (MeshWithDelays interface only)
  // Non-empty: write a VCD waveform of the ports and of every PE's state to this file (vcd.h).
  // Off by default; it only reads state and does not change timing (systolique_vcd).
  std::string vcd_path;
};

class SystolicArray {
 public:
  // Builds the array (and the token twin) at power-on: every register 0, as Verilator's
  // --x-initial 0; cycle() == 0. Opens the VCD file if requested.
  explicit SystolicArray(const ArrayConfig &cfg, ArrayOptions opt = {});
  ~SystolicArray();
  SystolicArray(const SystolicArray &) = delete;
  SystolicArray &operator=(const SystolicArray &) = delete;

  const ArrayConfig &config() const { return cfg_; }
  const ArrayOptions &options() const { return opt_; }
  bool has_provenance() const { return bool(twin_); }

  // Power-on state again (the components are rebuilt), then `cycles` cycles with reset high and
  // idle inputs (4, as the stored RTL traces); afterwards cycle() == 0.
  void reset(unsigned cycles = 4);
  int64_t cycle() const { return cycle_; }

  // MeshWithDelays interface (Interface::MeshWithDelays).
  // This cycle's outputs (ready, resp, tags), before its inputs are known.
  const MwdOut &out() const;
  // This cycle's inputs (in.reset: a cycle with reset high, MeshWithDelays.scala:251-253; it
  // restarts cycle numbering and the accounting; rows must not be in flight). `note` applies to
  // the request offered in this cycle if it fires. Inputs stay until set again.
  void set_inputs(const MwdIn &in, const RequestNote *note = nullptr);
  // set_inputs + tick; returns the outputs seen in that cycle.
  MwdOut step(const MwdIn &in, const RequestNote *note = nullptr);

  // Bare Mesh interface (Interface::Mesh): Mesh.scala's ports; ops (runs of rows with one id)
  // stand for requests; no provenance.
  const MeshOut &mesh_out() const;
  void set_mesh_inputs(const MeshIn &in);
  MeshOut step_mesh(const MeshIn &in);

  // The clock edge that ends the current cycle (either interface).
  void tick();

  // State.
  const Mesh &mesh() const;
  const MeshWithDelays &mesh_with_delays() const;  // MeshWithDelays interface only
  const PeRegs &regs(unsigned row, unsigned col) const { return mesh().pe(row, col); }
  PeView pe(unsigned row, unsigned col) const;
  const std::vector<RequestInfo> &requests() const { return requests_; }
  const std::vector<DRowInfo> &d_rows() const { return drows_; }
  // Rows fed into the Mesh since reset: request and row index within it.
  struct RowInfo {
    int request = -1;
    unsigned row = 0;
    int64_t cycle = -1;
  };
  const std::vector<RowInfo> &rows() const { return rows_[0]; }

  Accounting accounting() const;
  // Conservation of an Accounting: every PE-cycle in exactly one state (per cycle = pes, per PE
  // = cycles), at most one MAC per PE-cycle, per-request, per-cycle and per-PE sums equal the
  // totals, flags only on Mac PE-cycles. "" if all hold, else the first violation.
  static std::string check(const Accounting &a);

 public:
  struct Vcd;  // implementation detail: the VCD writer and its variable handles (systolic_array.cpp)

 private:
  struct Entry {  // occupied PE-cycles of one request in one cycle / at one PE
    int request;
    uint32_t count, result_out;
  };
  class Twin;

  void build();
  void clear_run();
  void tick_mwd();
  void tick_mesh();
  void account_mwd(const MwdIn &in, const MwdOut &o, const RequestNote *note);
  void account_mesh(const MeshIn &in);
  void account_pes();
  void after_edge();
  void declare_vcd();
  void sample_vcd();
  bool result_out(unsigned pe, unsigned shadow) const;
  void close_request(RequestInfo &r);
  RegSource decode(int64_t token) const;  // a twin token as a source
  void require(Interface i) const;

  ArrayConfig cfg_;
  ArrayOptions opt_;
  std::unique_ptr<MeshWithDelays> mwd_;  // MeshWithDelays interface (owns the Mesh)
  std::unique_ptr<Mesh> mesh_;           // bare Mesh interface
  std::unique_ptr<Twin> twin_;
  std::unique_ptr<Vcd> vcd_;
  MwdIn in_;
  MeshIn mesh_in_;
  bool has_note_ = false;
  RequestNote note_;
  int64_t cycle_ = 0;
  int64_t edges_ = 0;  // clock edges since construction (VCD time)

  std::vector<RequestInfo> requests_;
  std::vector<std::vector<RowInfo>> rows_;  // per input lane (one shared list with MeshWithDelays)
  std::vector<DRowInfo> drows_;
  uint64_t d_base_ = 0;         // d handshakes before the last reset (tokens keep counting)
  int64_t d_pending_ = -1;      // last d handshake not yet consumed
  uint64_t resp_rows_ = 0;      // response rows seen (row k of rows_ leaves as the k-th)
  std::vector<int> tag_fifo_;   // non-flush requests whose tag is queued (TagQueue order)
  size_t tag_head_ = 0;
  std::vector<int> mesh_op_of_id_;  // Mesh interface: latest op per id
  int mesh_open_op_ = -1;

  std::vector<uint64_t> pe_seen_;          // valid cycles per PE since reset
  struct ValidPe {  // a PE valid in the cycle being stepped
    unsigned pe, active;
    int request;
    int64_t twin_active;  // the twin's token in the active register before the edge
  };
  std::vector<ValidPe> pe_valid_now_;
  std::vector<std::array<RegSource, 2>> src_;
  std::vector<size_t> cyc_begin_;          // entries of cycle t: cyc_ent_[cyc_begin_[t] .. [t+1])
  std::vector<Entry> cyc_ent_;
  std::vector<std::vector<Entry>> pe_runs_;
};

}  // namespace systolique
