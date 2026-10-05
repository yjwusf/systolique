// Standalone elaboration of Gemmini's systolic array (Mesh, MeshWithDelays) to Verilog.
//
// Compiles the unmodified Gemmini sources listed below from GEMMINI_DIR (ucb-bar/gemmini v0.7.2,
// 709bc56) and berkeley-hardfloat from HARDFLOAT_DIR (9deaf1d, the commit rocket-chip pins in the
// Chipyard Gemmini v0.7.2 names), with the Scala and Chisel versions of Gemmini's own build.sbt
// (gemmini/build.sbt:7-10). Nothing from rocket-chip is needed: these files only use chisel3
// and hardfloat. src/GemminiTops.scala adds wiring-only wrappers and the generator main.
// Run through accelsim/gemmini/rtl/elaborate.sh.

val gemminiDir = file(sys.env.getOrElse("GEMMINI_DIR", sys.props("user.home") + "/opt/src/gemmini"))
val hardfloatDir = file(sys.env.getOrElse("HARDFLOAT_DIR",
  sys.props("user.home") + "/opt/src/berkeley-hardfloat-9deaf1d"))

// The Gemmini files the systolic array depends on (and nothing else).
val gemminiFiles = Seq("Arithmetic", "Dataflow", "Mesh", "MeshWithDelays", "PE", "Shifter",
  "SyncMem", "TagQueue", "Tile", "Transposer", "Util")

lazy val root = (project in file("."))
  .settings(
    name := "accelsim-gemmini-elab",
    scalaVersion := "2.13.10",
    libraryDependencies += "edu.berkeley.cs" %% "chisel3" % "3.6.0",
    addCompilerPlugin("edu.berkeley.cs" % "chisel3-plugin" % "3.6.0" cross CrossVersion.full),
    scalacOptions ++= Seq("-deprecation", "-unchecked", "-language:reflectiveCalls"),
    Compile / unmanagedSourceDirectories := Seq(baseDirectory.value / "src",
      hardfloatDir / "src" / "main" / "scala"),
    Compile / unmanagedSources ++= gemminiFiles.map(f =>
      gemminiDir / "src" / "main" / "scala" / "gemmini" / (f + ".scala")),
  )
