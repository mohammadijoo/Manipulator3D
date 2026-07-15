# frozen_string_literal: true

require "rubygems"
require "raylib"

# Ensure we load raylib shared libraries exactly once.
unless defined?($RAYLIB_BINDINGS_LOADED) && $RAYLIB_BINDINGS_LOADED
  shared_lib_path = Gem::Specification.find_by_name("raylib-bindings").full_gem_path + "/lib/"

  case RUBY_PLATFORM
  when /mswin|msys|mingw|cygwin/
    Raylib.load_lib(
      shared_lib_path + "libraylib.dll",
      raygui_libpath: shared_lib_path + "raygui.dll",
      physac_libpath: shared_lib_path + "physac.dll"
    )
  when /darwin/
    Raylib.load_lib(
      shared_lib_path + "libraylib.dylib",
      raygui_libpath: shared_lib_path + "raygui.dylib",
      physac_libpath: shared_lib_path + "physac.dylib"
    )
  when /linux/
    arch = RUBY_PLATFORM.split("-")[0]
    Raylib.load_lib(
      shared_lib_path + "libraylib.#{arch}.so",
      raygui_libpath: shared_lib_path + "raygui.#{arch}.so",
      physac_libpath: shared_lib_path + "physac.#{arch}.so"
    )
  else
    raise "Unknown OS: #{RUBY_PLATFORM}"
  end

  # Make raylib functions available as plain methods (InitWindow, BeginDrawing, ...)
  include Raylib

  $RAYLIB_BINDINGS_LOADED = true
end
