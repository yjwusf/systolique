// Standalone elaboration of Gemmini's ExecuteController with its scratchpad and accumulator banks
// (src/ExecuteTop.scala), of its command path (src/CmdTop.scala) and of both together
// (src/CtrlTop.scala) to Verilog, for the lockstep benches of the frontend classes.
//
// Compiles the unmodified Gemmini sources (GEMMINI_DIR, ucb-bar/gemmini v0.7.2, 709bc56) with
// the rocket-chip they were released against: rocket-chip 67ceb1d (the commit Chipyard ef3409f,
// Gemmini v0.7.2's CHIPYARD.hash, pins), its cde 384c06b and hardfloat 9deaf1d, at Gemmini's
// Scala and Chisel versions (gemmini/build.sbt:7-10). FireSim's midas.targetutils (PerfCounter,
// SynthesizePrintf: Gemmini only uses them with use_firesim_simulation_counters, false in its
// defaultConfig) is replaced by the no-op src/MidasTargetutils.scala. Run through
// rtl/ex/elaborate_ex.sh.

val home = sys.props("user.home")
def dir(env: String, default: String) = file(sys.env.getOrElse(env, home + "/opt/src/" + default))
val gemminiDir = dir("GEMMINI_DIR", "gemmini")
val rocketDir = dir("ROCKETCHIP_DIR", "rocket-chip-67ceb1d")
val cdeDir = dir("CDE_DIR", "cde-384c06b")
val hardfloatDir = dir("HARDFLOAT_DIR", "berkeley-hardfloat-9deaf1d")

val chiselVersion = "3.6.0"
lazy val common = Seq(
  scalaVersion := "2.13.10",
  scalacOptions ++= Seq("-deprecation", "-unchecked", "-language:reflectiveCalls", "-Ymacro-annotations"),
  libraryDependencies += "org.scala-lang" % "scala-reflect" % scalaVersion.value,
)

// rocket-chip's macros must be compiled before their users
lazy val macros = (project in file("macros"))
  .settings(common,
    Compile / unmanagedSourceDirectories := Seq(rocketDir / "macros" / "src" / "main" / "scala"))

lazy val root = (project in file("."))
  .dependsOn(macros)
  .settings(common,
    name := "systolique-gemmini-frontend-elab",
    libraryDependencies ++= Seq(
      "edu.berkeley.cs" %% "chisel3" % chiselVersion,
      "org.json4s" %% "json4s-native" % "4.0.6"),
    addCompilerPlugin("edu.berkeley.cs" % "chisel3-plugin" % chiselVersion cross CrossVersion.full),
    Compile / unmanagedSourceDirectories := Seq(
      baseDirectory.value / "src",
      hardfloatDir / "src" / "main" / "scala",
      cdeDir / "cde" / "src" / "chipsalliance" / "rocketchip",
      rocketDir / "src" / "main" / "scala"),
    // every Gemmini source but the Chipyard configurations (package chipyard)
    // rocket-chip's Verilog black boxes (plusarg_reader.v for the ReservationStation's timeout)
    Compile / unmanagedResourceDirectories += rocketDir / "src" / "main" / "resources",
    Compile / unmanagedSources ++=(gemminiDir / "src" / "main" / "scala" / "gemmini" * "*.scala").get
      .filterNot(f => Set("CustomCPUConfigs.scala", "CustomSoCConfigs.scala").contains(f.getName)),
  )
