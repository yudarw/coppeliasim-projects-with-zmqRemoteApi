sim = require('sim')

function sysCall_init()
    graph = sim.getObject('/object_velocity_graph')
    stream = sim.addGraphStream(graph, 'red cylinder', 'm/s', 0, {1, 0, 0})
end

function sysCall_sensing()
    sim.setGraphStreamValue(graph, stream, sim.getObjectVelocity(h)[3])
endP