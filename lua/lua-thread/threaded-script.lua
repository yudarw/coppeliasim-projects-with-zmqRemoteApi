function sysCall_init()
    print("From threaded Lua simulation script, init section")
    sim = require('sim')
    sim.setStepping(false) -- is actually in non-stepping by default
end

function sysCall_thread()
    print("From threaded Lua simulation script, thread section")
    cube = sim.getObject('..')
    pos = sim.getObjectPosition(cube)
    while not sim.getSimulationStopping() do
        pos[2] = pos[2] + 0.00002 -- we use a much smaller step, since this loop is executed very fast
        sim.setObjectPosition(cube, pos)
    end
    print("From threaded Lua simulation script, cleanup section")
end
