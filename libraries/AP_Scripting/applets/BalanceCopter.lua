-- Load CAN driver with a buffer size of 20
local driver = CAN:get_device(20)

local THIS_SCRIPT = "BalanceCopter.lua"
local UPDATE_INTERVAL_MS = 10

local send_msg = CANFrame()

local PARAM_TABLE_KEY = 72

assert(param:add_table(PARAM_TABLE_KEY, "BALA_", 5), 'could not add param BALA_')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'CUR_KP', 0), 'could not add CUR_KP')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'CUR_KI', 0), 'could not add CUR_KI')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'CUR_KD', 0), 'could not add CUR_KD')

assert(param:add_param(PARAM_TABLE_KEY, 4, 'ANG_KP', 3), 'could not add ANG_KP')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'ANG_KD', 5.7), 'could not add ANG_KD')

local control_current_kp = param:get('BALA_CUR_KP')
local control_current_ki = param:get('BALA_CUR_KI')
local control_current_ki_error_1 = 0
local control_current_ki_error_2 = 0

local left_wheel_speed = 0
local right_wheel_speed = 0

local cnt = 0
-- send a motor command
function set_current(target_ID, current)
    if (target_ID == 1)
    then
        send_msg:data(0, current >> 8)
        send_msg:data(1, current & 0xff)
    elseif (target_ID == 2)
    then
        send_msg:data(2, current >> 8)
        send_msg:data(3, current & 0xff)
    elseif (target_ID == 3)
    then
        send_msg:data(4, current >> 8)
        send_msg:data(5, current & 0xff)
    elseif (target_ID == 4)
    then
        send_msg:data(6, current >> 8)
        send_msg:data(7, current & 0xff)
    end
    -- sending 8 bytes of data
    send_msg:dlc(8)
    -- write the frame with a 10000us timeout
    driver:write_frame(send_msg, 10000)
end

-- send a motor command
function send_current()
    send_msg:id(512)
    send_msg:dlc(8)
    -- write the frame with a 10000us timeout
    driver:write_frame(send_msg, 10000)
end

function show_frame(dnum, frame)
    gcs:send_text(0,
        string.format("CAN[%u] msg from " .. tostring(frame:id()) .. ": %i, %i, %i, %i, %i, %i, %i, %i", dnum,
            frame:data(0), frame:data(1), frame:data(2), frame:data(3), frame:data(4), frame:data(5), frame:data(6),
            frame:data(7)))
end

function recv_speed()
    frame = driver:read_frame()

    if not frame then
        return
    end

    if ((frame:id()) == uint32_t(513))
    then
        left_wheel_speed = (frame:data(2) << 8) | (frame:data(3))
    end


    cnt = cnt + 1
    if (cnt > 100)
    then
        cnt = 0
        -- show_frame(2, frame)
        gcs:send_text(0, string.format("left_wheel_speed: %u", tostring(left_wheel_speed)))
    end
end

function control_speed(id, target_speed)
    if (id == 1)
    then
        local control_current_error_1 = (target_speed - left_wheel_speed)
        control_current_ki_error_1 = control_current_ki_error_1 + control_current_error_1
        local out1 = control_current_error_1 * math.tointeger(control_current_kp) + control_current_ki_error_1 * math.tointeger(control_current_ki)
        set_current(1, out1)
    elseif (id == 2)
    then
    end
end

function update()
    recv_speed()
    control_speed(1, 100)
    send_current()
    return update, UPDATE_INTERVAL_MS
end

gcs:send_text(0, "Starting BalanceCopter Control")

return update() -- run immediately before starting to reschedule
