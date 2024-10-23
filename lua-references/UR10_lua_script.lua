-- =======================================================
-- Remote API program for controlling arm robot (Updated)
-- June 25, 2022
-- @yudarw
-- =======================================================

-- Initialization:
function sysCall_init()
    corout=coroutine.create(coroutineMain)
    
    -- Get object handles:
    simTip=sim.getObject('./ikTip')
    simTarget=sim.getObject('./ikTarget')
    simRobot=sim.getObject('.')
    simTargetPos=sim.getObject('./targetPos')

    -- Additional sensor and actuator handles
    simGripper=sim.getObject('./RG2')
    simForceSensor=sim.getObject('./connector')
    --simSuction=sim.getObject('./vacuumGripper')
 
    -- Prepare an ik group:
    ikEnv=simIK.createEnvironment()
    ikGroup=simIK.createIkGroup(ikEnv)
    
    -- Method 1:
    -- simIK.addIkElementFromScene(ikEnv,ikGroup,robot,simTip,simTarget,simIK.constraint_pose)
    
    -- Method 2:
    simIK.setIkGroupCalculation(ikEnv,ikGroup,simIK.method_damped_least_squares,0.01,10)
    local ikElement=simIK.addIkElementFromScene(ikEnv,ikGroup,simRobot,simTip,simTarget,simIK.constraint_pose)
    simIK.setIkElementPrecision(ikEnv,ikGroup,ikElement,{0.0005,0.005*math.pi/180})
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

-- ======================================================
-- RemoteApi Functions
-- ======================================================
-- Set Robot Speed --
function remoteApi_setSpeed(inInt,inFloat,inString,inBuffe)
    local linear_vel=inFloat[1]
    local angular_vel=inFloat[2]
    local ikMaxVel={linear_vel, linear_vel, linear_vel, angular_vel}
    local fkMaxVel={angular_vel, angular_vel, angular_vel, angular_vel, angular_vel, angular_vel}
    print('Remote API -> Set robot speed = ', ikMaxVel)
    ik_data.maxVel = ikMaxVel   
    fk_data.maxVel = fkMaxVel 
end

-- Move Robot Position --
function remoteApi_movePosition(inInt,inFloat,inString,inBuffer)
    targetPos={}
    targetPos[1]=inFloat[1]
    targetPos[2]=inFloat[2]
    targetPos[3]=inFloat[3]
    targetPos[4]=inFloat[4]
    targetPos[5]=inFloat[5]
    targetPos[6]=inFloat[6]
    moveIkMode=true
end

-- Move Robot Joint Position --
function remoteApi_moveJointPosition(inInt,inFloat,inString,inBuffer)
    targetJoint={}
    for i=1,6,1 do
        targetJoint[i] = inFloat[i] 
    end
    moveJointMode=true
end

-- Get Object Position --
function remoteApi_getPosition()
    --val = sim.getFloatSignal('forceX')
    local pos=sim.getObjectPosition(simTip, simRobot)
    local rot=sim.getObjectOrientation(simTip, simRobot)
    local endpos = {pos[1],pos[2],pos[3],rot[1],rot[2],rot[3]}
    return {},endpos,{},''
end

-- Get Object Joint Position --
function remoteApi_getJointPosition()
    local jointPos={}
    for i=1,6,1 do
        jointPos[i]=sim.getJointPosition(simJoints[i])
    end
    return {},jointPos,{},''
end

-- Set Gripper --
function remoteApi_setGripper(state)
    velocity=0.11
    force=20
    if state[1]==0 then
        velocity=-velocity
    end
    local dat={}
    dat.velocity=velocity
    dat.force=force
    sim.writeCustomDataBlock(simGripper,'activity',sim.packTable(dat))
end

-- Set Gripper On--
function remoteApi_setGripperOn()
    print('RemoteAPI set gripper ON')
    sim.setStringSignal('gripperState','ON')
    end
-- Set Gripper Off--
function remoteApi_setGripperOff()
    print('RemoteAPI set gripper OFF')
    sim.setStringSignal('gripperState','OFF')
end

function remoteApi_suctionOn()
    print('> Robot set suction ON')
    sim.writeCustomDataBlock(simSuction,'suction','on')
end

function remoteApi_suctionOff()
    print('> Robot set suction OFF')
    sim.writeCustomDataBlock(simSuction,'suction','off')
end

-- ========================================================
-- Kinematic Functions
-- ========================================================

-- Inverse Kinematic --
function moveToPoseCallback(q,velocity,accel,auxData)
    sim.setObjectPose(simTarget,-1,q)
    simIK.applyIkEnvironmentToScene(auxData.ikEnv,auxData.ikGroup)
end

function moveToPose_viaIK(auxData, pose)
    local pos = {pose[1], pose[2], pose[3]}
    local ori = {pose[4], pose[5], pose[6]}
    sim.setObjectPosition(simTargetPos, simRobot, pos)
    sim.setObjectOrientation(simTargetPos, simRobot, ori)
    local targetQ  = sim.getObjectPose(simTargetPos,-1)
    local currentQ = sim.getObjectPose(simTip,-1)
    return sim.moveToPose(-1,currentQ,auxData.maxVel,auxData.maxAccel,auxData.maxJerk,targetQ,moveToPoseCallback,auxData,nil)
end

-- Forward Kinematic --
function moveToConfigCallback(config,velocity,accel,auxData)
    for i=1,#auxData.joints,1 do
        local jh=auxData.joints[i]
        if sim.isDynamicallyEnabled(jh) then
            sim.setJointTargetPosition(jh,config[i])
        else    
            sim.setJointPosition(jh,config[i])
        end
    end
end

function moveToConfig_viaFK(auxData, goalConfig)
    local startConfig={}
    for i=1,#auxData.joints,1 do
        startConfig[i]=sim.getJointPosition(auxData.joints[i])
    end
    sim.moveToConfig(-1,startConfig,nil,nil,auxData.maxVel,auxData.maxAccel,auxData.maxJerk,goalConfig,nil,moveToConfigCallback,auxData,nil)
end


-- ============================================================== 
-- Local command called inside the Simulator Editor
-- ==============================================================
-- Local set position --
function setPosition(auxData, pos) 
    local pose={}
    pose[1]=pos[1] / 1000
    pose[2]=pos[2] / 1000
    pose[3]=pos[3] / 1000
    pose[4]=pos[4] * math.pi / 180
    pose[5]=pos[5] * math.pi / 180
    pose[6]=pos[6] * math.pi / 180
    print(pose)
    moveToPose_viaIK(auxData, pose)
end

-- Local set joint position --
function setJointPosition(auxData, config)
    goalConfig={}
    for i=1,6,1 do
        goalConfig[i]=config[i] * math.pi / 180
    end
    moveToConfig_viaFK(auxData, goalConfig)
end

-- Local set gripper --
function setGripperOpen(state)
    if state then
        --remoteApi_setGripper({1})
        sim.setStringSignal('gripperState','ON')
    else
        --remoteApi_setGripper({0})
        sim.setStringSignal('gripperState','OFF')        
    end
end

-- Local set speed --
function set_speed(sp)
    local ang_vel = 10 * math.pi / 180
    local lin_vel = sp / 1000
    local vel={lin_vel, ang_vel}
    remoteApi_setSpeed({},vel,{},'')
end


-- ==================================================
-- Examples
-- ==================================================
function ik_command_demo()
	--workpiece2Handle=sim.getObject('/workpiece2')
    --orkpiece_pos = sim.getObjectPosition(workpiece2Handle,simRobot)

    -- Test IK Movement:
    pos1={500, -300, 100, 180, 0, 0}
    pos2={500, -300, 200, 180, 0, 0}
    pos3={500,  300, 200, 180, 0, 0}
    pos4={500,  300, 100, 180, 0, 0}
    
    --setGripperOpen(true)
    
    while true do
        setPosition(ik_data, pos2)
        setPosition(ik_data, pos1)
        --setGripperOpen(false)
        sim.wait(1)
        setPosition(ik_data, pos2)
    
        setPosition(ik_data, pos3)
        setPosition(ik_data, pos4)
        --setGripperOpen(true)
        sim.wait(1)
        setPosition(ik_data, pos3)
    end
end


function fk_command_demo()
    conf1={0, 0, 0, 0, 0, 0}
    conf2={0, -15, 90, 90, 45, 0}
    conf3={115, -15, 90, 90, 45, 0}
    conf4={0, 0, 0, 0, 0, 0}

    while true do
        setJointPosition(fk_data, conf1)
        setJointPosition(fk_data, conf2)
        setJointPosition(fk_data, conf3)
        setJointPosition(fk_data, conf4)
    end
end

-- ==================================================
--                         Main
-- ==================================================
function coroutineMain()

    simJoints={}
    for i=1,6,1 do
        simJoints[i]=sim.getObject('./joint'..i)
    end
    
    -- FK movement data:
    local initConf={0,0,0,0,0,0}
    local vel=180
    local accel=40
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    local initConf={}
    local maxConfVel={}
    local maxConfAccel={}
    local maxConfJerk={}
    for i=1,#simJoints,1 do
        initConf[i]=sim.getJointPosition(simJoints[i])
        maxConfVel[i]=3*math.pi/180
        maxConfAccel[i]=1.15
        maxConfJerk[i]=0.4
    end
    
    fk_data={}
    fk_data.maxVel=maxConfVel
    fk_data.maxAccel=maxConfAccel
    fk_data.maxJerk=maxConfJerk
    fk_data.joints=simJoints
    

    -- IK movement data:
    local ikMaxVel={1.0,1.0,1.0,1000*3.14/180}
    local ikMaxAccel={1.0,1.0,1.0,1000*3.14/180}
    local ikMaxJerk={2.0,2.0,2.0,2000*3.14/180}
    
    ik_data={}
    ik_data.maxVel=ikMaxVel
    ik_data.maxAccel=ikMaxAccel
    ik_data.maxJerk=ikMaxJerk
    ik_data.ikEnv=ikEnv
    ik_data.ikGroup=ikGroup

    set_speed(100)

    -- Test IK Movement:
    --ik_command_demo()
    --fk_command_demo()
   
    --sim.wait(5)
    --sim.stopSimulation()
   
    
    while true do
        if moveIkMode then
            moveIkMode=false
            sim.setInt32Signal('moving_status',1)           -- Trigger for C++
            sim.setStringSignal('moving_signal', 'MOVING')  -- Trigger for Python
            moveToPose_viaIK(ik_data, targetPos)
            sim.setInt32Signal('moving_status',0)
            sim.setStringSignal('moving_signal', 'NOT_MOVING')
        end
        
        if moveJointMode then
            moveJointMode=false
            sim.setInt32Signal('moving_status',1)   
            sim.setStringSignal('moving_signal', 'MOVING')
            moveToConfig_viaFK(fk_data, targetJoint)
            sim.setInt32Signal('moving_status',0)
            sim.setStringSignal('moving_signal', 'NOT_MOVING')
        end
    end
end