function sysCall_init()
    print("From non-threaded Lua simulation script, init section")
    sim = require('sim')
    cube = sim.getObject('..')
    pos = sim.getObjectPosition(cube)
end

function sysCall_cleanup()
    print("From non-threaded Lua simulation script, cleanup section")
end

function sysCall_actuation()
    print("From non-threaded Lua simulation script, actuation section")
    pos[2] = pos[2] + 0.01
    sim.setObjectPosition(cube, pos)
end

function sysCall_sensing()
    print("From non-threaded Lua simulation script, sensing section")
end
