--[[
 Script to use LED strips as position lights.
 For this script we will use two strips with up to 8 LEDs each.
--]]
local num_leds = 3
local timer = 0

-- Brightness for green or red light.
local br_color_r = 255
local br_color_g = 255
local br_color_b = 255

-- Brightness for flash light when armed.
local br_flash = 255

--[[
 Use SERVOn_FUNCTION 94 for left LED strip
 Use SERVOn_FUNCTION 95 for right LED strip
--]]
local chan_left = assert(SRV_Channels:find_channel(94), "LEDs left: channel not set")

-- find_channel returns 0 to 15, convert to 1 to 16
chan_left = chan_left + 1

gcs:send_text(6, "LEDs strip left: chan=" .. tostring(chan_left))

-- initialisation code
assert(serialLED:set_num_neopixel(chan_left, num_leds), "Failed left LED setup")

function update_LEDs()
  if arming:is_armed() then
    if (timer == 0) then
      br_color_r = 0
      br_color_g = 255
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    elseif (timer == 1) then
      br_color_r = 0
      br_color_g = 0
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    elseif (timer == 2) then
      br_color_r = 0
      br_color_g = 255
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    elseif (timer == 3) then
      br_color_r = 0
      br_color_g = 0
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    end
    timer = timer + 1
    if (timer > 10) then
      timer = 0
    end
  else
    if (timer == 4) then
      br_color_r = 0
      br_color_g = 255
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    elseif (timer == 8) then
      br_color_r = 0
      br_color_g = 0
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    elseif (timer == 12) then
      br_color_r = 0
      br_color_g = 255
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    elseif (timer == 16) then
      br_color_r = 0
      br_color_g = 0
      br_color_b = 0
      serialLED:set_RGB(chan_left, -1, br_color_r, br_color_g, br_color_b)
    end
    timer = timer + 1
    if (timer > 20) then
      timer = 0
    end
  end
  serialLED:send(chan_left)
  return update_LEDs, 100 -- run at 10Hz
end

return update_LEDs()