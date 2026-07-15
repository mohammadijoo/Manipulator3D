ThisBuild / scalaVersion := "3.3.7"
ThisBuild / organization := "mohammadijoo"
ThisBuild / version      := "1.0.0"

lazy val root = (project in file(".")).settings(
  name := "Manipulator3D",

  // jaylib (raylib bindings)
  libraryDependencies += "uk.co.electronstudio.jaylib" % "jaylib" % "5.5.0-2",

  // Run in a forked JVM so encoding/locale flags apply cleanly
  Compile / run / fork := true,

  // Force English + UTF-8 for predictable formatting across machines
  Compile / run / javaOptions ++= Seq(
    "-Dfile.encoding=UTF-8",
    "-Dsun.jnu.encoding=UTF-8",
    "-Duser.language=en",
    "-Duser.country=US"
  )
)
