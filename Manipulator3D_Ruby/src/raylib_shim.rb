# frozen_string_literal: true

# A small compatibility layer for raylib Ruby bindings.
# It resolves function calls across:
# - snake_case vs CamelCase names
# - functions exposed as module singleton methods vs instance methods
# - functions located in nested modules under Raylib
#
# Usage:
#   require "raylib"
#   require_relative "raylib_shim"
#   RL.init_window(800, 450, "Hello")
#
# Constants and structs are still accessed via Raylib::...

module RL
  module_function

  def camelize(sym)
    sym.to_s.split("_").map(&:capitalize).join.to_sym
  end

  def variants(sym)
    s = sym.to_sym
    out = [s]
    out << :"#{s}?" unless s.to_s.end_with?("?")
    out << camelize(s)
    out
  end

  def module_providers
    @module_providers ||= begin
      mods = []
      mods << Raylib if defined?(Raylib)

      if defined?(Raylib)
        Raylib.constants.each do |c|
          v = Raylib.const_get(c) rescue nil
          mods << v if v.is_a?(Module)
        end
      end

      mods.compact.uniq
    end
  end

  def instance_provider
    @instance_provider ||= begin
      obj = Object.new
      obj.extend(Raylib) if defined?(Raylib)
      obj
    end
  end

  def available?(name)
    sym = name.to_sym
    vs = variants(sym)

    module_providers.each do |m|
      return true if vs.any? { |v| m.respond_to?(v) }
    end

    inst = instance_provider
    return true if vs.any? { |v| inst.respond_to?(v) }

    false
  end

  def dispatch(name, *args, &blk)
    sym = name.to_sym
    vs = variants(sym)

    module_providers.each do |m|
      v = vs.find { |meth| m.respond_to?(meth) }
      return m.public_send(v, *args, &blk) if v
    end

    inst = instance_provider
    v = vs.find { |meth| inst.respond_to?(meth) }
    return inst.public_send(v, *args, &blk) if v

    raise NoMethodError,
          "Raylib API not found: #{sym} (tried #{vs.map(&:to_s).join(', ')})"
  end

  def method_missing(name, *args, &blk)
    dispatch(name, *args, &blk)
  end

  def respond_to_missing?(name, include_private = false)
    available?(name) || super
  end
end
