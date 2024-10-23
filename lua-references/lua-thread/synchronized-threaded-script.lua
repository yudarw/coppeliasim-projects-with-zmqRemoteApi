function sysCall_init()
    print("From synchronized threaded Lua simulation script, init section")
    sim = require('sim')
    sim.setStepping(true)
end

function sysCall_thread()
    print("From synchronized threaded Lua simulation script, thread section")
    cube = sim.getObject('..')
    pos = sim.getObjectPosition(cube)
    while not sim.getSimulationStopping() do
        pos[2] = pos[2] + 0.01
        sim.setObjectPosition(cube, pos)
        sim.step() -- Allow CoppeliaSim to switch to another thread
    end
    print("From synchronized threaded Lua simulation script, cleanup section")
end
