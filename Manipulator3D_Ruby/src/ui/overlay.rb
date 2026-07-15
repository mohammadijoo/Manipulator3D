# frozen_string_literal: true

require_relative "../raylib_bootstrap"
require_relative "../util/math3d"
require_relative "../render/draw_utils"

module UI
  class Overlay
    LINE_SCALE = 1.20

    def initialize(font)
      @font = font
      @focus_key = nil
      @blink_t = 0.0

      @start_fields = [
        { key: :sx, label: "x", value: "1" },
        { key: :sy, label: "y", value: "2" },
        { key: :sz, label: "z", value: "1" }
      ]
      @goal_fields = [
        { key: :gx, label: "x", value: "2" },
        { key: :gy, label: "y", value: "3" },
        { key: :gz, label: "z", value: "2" }
      ]
    end

    def set_fields_from_vectors(start_v, goal_v)
      @start_fields[0][:value] = format("%.3f", start_v.x)
      @start_fields[1][:value] = format("%.3f", start_v.y)
      @start_fields[2][:value] = format("%.3f", start_v.z)

      @goal_fields[0][:value] = format("%.3f", goal_v.x)
      @goal_fields[1][:value] = format("%.3f", goal_v.y)
      @goal_fields[2][:value] = format("%.3f", goal_v.z)
    end

    def draw(arm:, paused:, start_current:, goal_current:, phase_text:, runtime_error:, screen_w:, screen_h:)
      pad = 12
      x0 = 14
      y0 = 14
      w  = 360
      h  = 560 # +100 px as requested (was 460)

      DrawRectangle(x0, y0, w, h, Color.from_u8(18, 18, 18, 230))
      DrawRectangleLines(x0, y0, w, h, Color.from_u8(200, 200, 200, 255))

      y = y0 + pad
      Render.draw_text_bold(@font, "3-DOF 2-Link Arm", x0 + pad, y, 24, RAYWHITE)
      y += inc(30)

      if phase_text && !phase_text.empty?
        Render.draw_text_small(@font, phase_text, x0 + pad, y, 18, SKYBLUE)
        y += inc(24)
      end

      # PLAY/PAUSE button
      btn = Rectangle.create((x0 + pad).to_f, y.to_f, (w - 2 * pad).to_f, 34.0)
      btn_bg = paused ? Color.from_u8(60, 120, 60, 220) : Color.from_u8(120, 60, 60, 220)

      DrawRectangleRounded(btn, 0.18, 8, btn_bg)
      draw_rounded_outline(btn, 0.18, 8, 2.0, Color.from_u8(230, 230, 230, 255))

      label = paused ? "PLAY (apply inputs)" : "PAUSE"
      Render.draw_text_bold(@font, label, btn.x.to_i + 10, btn.y.to_i + 7, 18, RAYWHITE)

      mp = GetMousePosition()
      clicked_btn = IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && point_in_rect?(mp, btn)

      request_apply = false
      paused = !paused if clicked_btn
      request_apply = true if clicked_btn && !paused

      y += inc(46)

      l1 = arm.link1
      l2 = arm.link2

      Render.draw_text_small(@font, format("Link1 length: %.3fm", l1.length_m), x0 + pad, y, 18, RAYWHITE); y += inc(20)
      Render.draw_text_small(@font, format("Link1 mass  : %.3fkg", l1.mass_kg), x0 + pad, y, 18, RAYWHITE); y += inc(20)
      Render.draw_text_small(@font, format("Link1 inertia (joint): %.5f", l1.inertia_joint), x0 + pad, y, 18, RAYWHITE); y += inc(24)

      Render.draw_text_small(@font, format("Link2 length: %.3fm", l2.length_m), x0 + pad, y, 18, RAYWHITE); y += inc(20)
      Render.draw_text_small(@font, format("Link2 mass  : %.3fkg", l2.mass_kg), x0 + pad, y, 18, RAYWHITE); y += inc(20)
      Render.draw_text_small(@font, format("Link2 inertia (joint): %.5f", l2.inertia_joint), x0 + pad, y, 18, RAYWHITE); y += inc(26)

      Render.draw_text_small(
        @font,
        format("Workspace |p|: [%.2f, %.2f]m", arm.min_reach, arm.max_reach),
        x0 + pad, y, 18, Color.from_u8(140, 200, 255, 255)
      )
      y += inc(26)

      Render.draw_text_bold(@font, "Inputs (edit when paused)", x0 + pad, y, 20, RAYWHITE)
      y += inc(26)

      input_area_x = x0 + pad
      input_w = w - 2 * pad
      field_w = (input_w - 60) / 3.0
      field_h = 30.0
      gap = 8.0

      y = draw_vec3_fields("START", input_area_x, y, field_w, field_h, gap, @start_fields, paused)
      y += inc(10)
      y = draw_vec3_fields("GOAL ", input_area_x, y, field_w, field_h, gap, @goal_fields, paused)

      y += inc(16)

      start_candidate, goal_candidate, parse_error = parse_candidates

      start_ok = false
      goal_ok = false
      start_msg = ""
      goal_msg = ""

      unless parse_error
        iks = arm.solve_ik(start_candidate, elbow_up: false)
        ikg = arm.solve_ik(goal_candidate, elbow_up: false)
        start_ok = iks.reachable
        goal_ok = ikg.reachable
        start_msg = iks.message
        goal_msg = ikg.message
      end

      Render.draw_text_small(
        @font,
        "START: " + (parse_error ? "invalid" : (start_ok ? "reachable" : "NOT reachable")),
        x0 + pad, y, 18, (parse_error ? ORANGE : (start_ok ? GREEN : ORANGE))
      )
      y += inc(20)
      unless start_ok || parse_error
        Render.draw_text_small(@font, "  reason: #{start_msg}", x0 + pad, y, 16, ORANGE)
        y += inc(20)
      end

      Render.draw_text_small(
        @font,
        "GOAL : " + (parse_error ? "invalid" : (goal_ok ? "reachable" : "NOT reachable")),
        x0 + pad, y, 18, (parse_error ? ORANGE : (goal_ok ? GREEN : ORANGE))
      )
      y += inc(20)
      unless goal_ok || parse_error
        Render.draw_text_small(@font, "  reason: #{goal_msg}", x0 + pad, y, 16, ORANGE)
        y += inc(20)
      end

      if parse_error
        Render.draw_text_bold(@font, "INPUT ERROR!", x0 + pad, y, 20, RED); y += inc(22)
        Render.draw_text_small(@font, parse_error, x0 + pad, y, 16, RED); y += inc(20)
      end

      Render.draw_text_bold(@font, runtime_error.to_s, x0 + pad, y, 18, RED) if runtime_error

      applied = nil
      if request_apply
        if parse_error
          applied = { ok: false, message: parse_error }
        else
          applied = {
            ok: (start_ok && goal_ok),
            start: start_candidate,
            goal: goal_candidate,
            message: (start_ok && goal_ok) ? nil : "Start/Goal out of reach."
          }
        end
      end

      { paused: paused, apply: applied }
    end

    private

    def inc(px)
      (px.to_f * LINE_SCALE).round
    end

    def draw_rounded_outline(rec, roundness, segments, thickness, color)
      begin
        DrawRectangleRoundedLinesEx(rec, roundness, segments, thickness, color)
      rescue StandardError
        begin
          DrawRectangleRoundedLines(rec, roundness, segments, thickness, color)
        rescue StandardError
          DrawRectangleLines(rec.x.to_i, rec.y.to_i, rec.width.to_i, rec.height.to_i, color)
        end
      end
    end

    def point_in_rect?(p, r)
      p.x >= r.x && p.x <= r.x + r.width && p.y >= r.y && p.y <= r.y + r.height
    end

    def accept_char?(ch)
      ch =~ /[0-9\.\-\+eE]/
    end

    def draw_vec3_fields(title, x, y, field_w, field_h, gap, fields, allow_edit)
      Render.draw_text_small(@font, "#{title} (x y z):", x, y, 18, RAYWHITE)
      y += inc(20)

      boxes_y = y
      label_w = 44.0

      mouse = GetMousePosition()
      clicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT)

      fields.each_with_index do |f, i|
        bx = x + label_w + i * (field_w + gap)
        r = Rectangle.create(bx.to_f, boxes_y.to_f, field_w.to_f, field_h.to_f)

        @focus_key = f[:key] if clicked && point_in_rect?(mouse, r) && allow_edit
        focused = (@focus_key == f[:key]) && allow_edit

        bg = focused ? Color.from_u8(35, 35, 40, 255) : Color.from_u8(28, 28, 32, 255)
        DrawRectangleRounded(r, 0.18, 6, bg)
        draw_rounded_outline(r, 0.18, 6, 2.0, focused ? SKYBLUE : Color.from_u8(160, 160, 160, 120))

        Render.draw_text_small(@font, f[:label], bx.to_i + 4, boxes_y.to_i - inc(18), 16, Color.from_u8(180, 180, 180, 255))

        text = f[:value].dup
        if focused
          @blink_t += GetFrameTime()
          text += "|" if (@blink_t % 1.0) < 0.5
        end
        Render.draw_text_small(@font, text, bx.to_i + 8, boxes_y.to_i + 6, 18, RAYWHITE)
      end

      if allow_edit && @focus_key
        edit_current_field

        if IsKeyPressed(KEY_TAB)
          all = (@start_fields + @goal_fields)
          idx = all.index { |h| h[:key] == @focus_key } || 0
          @focus_key = all[(idx + 1) % all.size][:key]
        end

        @focus_key = nil if IsKeyPressed(KEY_ENTER)
      end

      boxes_y + field_h + inc(10)
    end

    def edit_current_field
      all = (@start_fields + @goal_fields)
      field = all.find { |h| h[:key] == @focus_key }
      return unless field

      if IsKeyPressed(KEY_BACKSPACE)
        field[:value] = field[:value][0...-1] unless field[:value].empty?
      end

      field[:value] = "" if IsKeyPressed(KEY_DELETE)

      loop do
        code = GetCharPressed()
        break if code == 0

        ch = (code.chr(Encoding::UTF_8) rescue nil)
        next if ch.nil?
        next unless accept_char?(ch)

        field[:value] = (field[:value] + ch)[0, 16]
      end
    end

    def parse_float(s)
      t = s.to_s.strip
      return nil if t.empty?
      Float(t)
    rescue StandardError
      nil
    end

    def parse_candidates
      sx = parse_float(@start_fields[0][:value])
      sy = parse_float(@start_fields[1][:value])
      sz = parse_float(@start_fields[2][:value])

      gx = parse_float(@goal_fields[0][:value])
      gy = parse_float(@goal_fields[1][:value])
      gz = parse_float(@goal_fields[2][:value])

      if [sx, sy, sz, gx, gy, gz].any?(&:nil?)
        return [Vector3.create, Vector3.create, "Invalid format. Use numbers like: 1, -2.5, 0.75"]
      end

      if sz < 0.0 || gz < 0.0
        return [Vector3.create, Vector3.create, "Invalid input: z must be >= 0"]
      end

      [Vector3.create(sx, sy, sz), Vector3.create(gx, gy, gz), nil]
    end
  end
end
