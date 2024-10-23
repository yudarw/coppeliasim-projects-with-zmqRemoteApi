sim=require'sim'
simIK=require'simIK'

function sysCall_init()
    -- Following for the movement of the vehicle, i.e. locomotion:
    vehicleReference=sim.getObject('../youBot_vehicleReference')
    vehicleTarget=sim.getObject('../youBot_vehicleTargetPosition')
    sim.setObjectPosition(vehicleTarget,{0,0,0},sim.handle_parent)
    sim.setObjectOrientation(vehicleTarget,{0,0,0},sim.handle_parent)
    sim.setObjectParent(vehicleTarget,-1,true)
    --Prepare initial values and retrieve handles:
    wheelJoints={-1,-1,-1,-1} -- front left, rear left, rear right, front right
    wheelJoints[1]=sim.getObject('../rollingJoint_fl')
    wheelJoints[2]=sim.getObject('../rollingJoint_rl')
    wheelJoints[3]=sim.getObject('../rollingJoint_rr')
    wheelJoints[4]=sim.getObject('../rollingJoint_fr')
    previousForwBackVel=0
    previousLeftRightVel=0
    previousRotVel=0
end

function sysCall_actuation()
    -- Following for the movement of the vehicle, i.e. locomotion:
    relP=sim.getObjectPosition(vehicleTarget,vehicleReference)
    relE=sim.getObjectOrientation(vehicleTarget,vehicleReference)
    pParam=20
    maxV=2
    pParamRot=10
    maxVRot=3
    accelF=0.035
    forwBackVel=relP[2]*pParam
    leftRightVel=relP[1]*pParam
    v=math.sqrt(forwBackVel*forwBackVel+leftRightVel*leftRightVel)
    if v>maxV then
        forwBackVel=forwBackVel*maxV/v
        leftRightVel=leftRightVel*maxV/v
    end
    rotVel=-relE[3]*pParamRot
    if (math.abs(rotVel)>maxVRot) then
        rotVel=maxVRot*rotVel/math.abs(rotVel)
    end
    
    df=forwBackVel-previousForwBackVel
    ds=leftRightVel-previousLeftRightVel
    dr=rotVel-previousRotVel
    
    if (math.abs(df)>maxV*accelF) then
        df=math.abs(df)*(maxV*accelF)/df
    end
    
    if (math.abs(ds)>maxV*accelF) then
        ds=math.abs(ds)*(maxV*accelF)/ds
    end
    
    if (math.abs(dr)>maxVRot*accelF) then
        dr=math.abs(dr)*(maxVRot*accelF)/dr
    end
    
    
    forwBackVel=previousForwBackVel+df
    leftRightVel=previousLeftRightVel+ds
    rotVel=previousRotVel+dr
    
    sim.setJointTargetVelocity(wheelJoints[1],-forwBackVel-leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[2],-forwBackVel+leftRightVel-rotVel)
    sim.setJointTargetVelocity(wheelJoints[3],-forwBackVel-leftRightVel+rotVel)
    sim.setJointTargetVelocity(wheelJoints[4],-forwBackVel+leftRightVel+rotVel)
    
    previousForwBackVel=forwBackVel
    previousLeftRightVel=leftRightVel
    previousRotVel=rotVel
end

function getConfig(handles)
    local c={}
    for i=1,#handles,1 do
        c[i]=sim.getJointPosition(handles[i])
    end
    return c
end

openGripper=function()
    sim.writeCustomStringData(gripper,'activity','open')
    sim.wait(0.8)
end

closeGripper=function()
    sim.writeCustomStringData(gripper,'activity','close')
    sim.wait(0.8)
end

setGripperTargetMovingWithVehicle=function()
    sim.setObjectParent(gripperTarget,vehicleReference,true)
end

setGripperTargetFixedToWorld=function()
    sim.setObjectParent(gripperTarget,-1,true)
end

waitToReachVehicleTargetPositionAndOrientation=function()
    repeat
        sim.step() -- don't waste your time waiting!
        p1=sim.getObjectPosition(vehicleTarget)
        p2=sim.getObjectPosition(vehicleReference)
        p={p2[1]-p1[1],p2[2]-p1[2]}
        pError=math.sqrt(p[1]*p[1]+p[2]*p[2])
        oError=math.abs(sim.getObjectOrientation(vehicleReference,vehicleTarget)[3])
    until (pError<0.005)and(oError<0.5*math.pi/180) 
end

getBoxAdjustedMatrixAndFacingAngle=function(boxHandle)
    p2=sim.getObjectPosition(boxHandle)
    p1=sim.getObjectPosition(vehicleReference)
    p={p2[1]-p1[1],p2[2]-p1[2],p2[3]-p1[3]}
    pl=math.sqrt(p[1]*p[1]+p[2]*p[2]+p[3]*p[3])
    p[1]=p[1]/pl
    p[2]=p[2]/pl
    p[3]=p[3]/pl
    m=sim.getObjectMatrix(boxHandle)
    matchingScore=0
    for i=1,3,1 do
        v={m[0+i],m[4+i],m[8+i]}
        score=v[1]*p[1]+v[2]*p[2]+v[3]*p[3]
        if (math.abs(score)>matchingScore) then
            s=1
            if (score<0) then s=-1 end
            matchingScore=math.abs(score)
            bestMatch={v[1]*s,v[2]*s,v[3]*s}
        end
    end
    angle=math.atan2(bestMatch[2],bestMatch[1])
    m=sim.buildMatrix(p2,{0,0,angle})
    return m, angle-math.pi/2
end

pickupBoxFromPlace=function(boxHandle,pickupConf)
    local m,angle=getBoxAdjustedMatrixAndFacingAngle(boxHandle)
    sim.setObjectPosition(vehicleTarget,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
    sim.setObjectOrientation(vehicleTarget,{0,0,angle})
    local params = {
        joints = armJoints,
        targetPos = pickupConf,
        maxVel = fkSpeed,
        maxAccel = fkAccel,
        maxJerk = fkJerk,
    }
    sim.moveToConfig(params)
    waitToReachVehicleTargetPositionAndOrientation()
    setGripperTargetFixedToWorld()
    local q=sim.getObjectPose(gripperTip)
    q[1]=m[4]
    q[2]=m[8]
    local ikparams = {
        pose = sim.getObjectPose(gripperTip),
        targetPose = q,
        maxVel = ikSpeed,
        maxAccel = ikAccel,
        maxJerk = ikJerk,
        callback = ikcb2,
        auxData = ikData,
        metric = ikMetric
    }
    sim.moveToPose(ikparams)
    openGripper()
    q[3]=m[12]
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)
    closeGripper()
    q[3]=q[3]+0.1
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)
    setGripperTargetMovingWithVehicle()    
end

dropToPlatform=function(platform)
    local params = {
        joints = armJoints,
        targetPos = platform,
        maxVel = fkSpeed,
        maxAccel = {0.3,0.3,0.3,0.3,0.3},
        maxJerk = fkJerk,
    }
    sim.moveToConfig(params)
    openGripper()
end

pickupFromPlatformAndReorient=function(boxHandle)
    local params = {
        joints = armJoints,
        targetPos = platformIntermediateDrop,
        maxVel = fkSpeed,
        maxAccel = fkAccel,
        maxJerk = fkJerk,
    }
    sim.moveToConfig(params)
    local q=sim.getObjectPose(boxHandle)
    local ikparams = {
        pose = sim.getObjectPose(gripperTip),
        targetPose = q,
        maxVel = ikSpeed,
        maxAccel = ikAccel,
        maxJerk = ikJerk,
        callback = ikcb1,
        auxData = ikData,
        metric = ikMetric
    }
    sim.moveToPose(ikparams)
    closeGripper()
    -- Move a bit back from current position:
    local m=sim.getObjectMatrix(vehicleTarget)
    sim.setObjectPosition(vehicleTarget,{m[4]-m[2]*dist1,m[8]-m[6]*dist1,0})
    -- Now drop it
    params.pos = getConfig(armJoints)
    params.targetPos = pickup2
    sim.moveToConfig(params)
    openGripper()
    sim.wait(1)
    -- Now orient yourself according to the box and pick it up:
    local m,angle=getBoxAdjustedMatrixAndFacingAngle(boxHandle)
    sim.setObjectPosition(vehicleTarget,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
    sim.setObjectOrientation(vehicleTarget,{0,0,angle})
    waitToReachVehicleTargetPositionAndOrientation()
    params.pos = getConfig(armJoints)
    params.targetPos = pickup2
    sim.moveToConfig(params)
    setGripperTargetFixedToWorld()
    local q=sim.getObjectPose(gripperTip)
    q[1]=m[4]
    q[2]=m[8]
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    ikparams.callback = ikcb2
    sim.moveToPose(ikparams)
    q[3]=0.03
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)
    closeGripper()
    q[3]=q[3]+0.1
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)
    setGripperTargetMovingWithVehicle()
end

dropToPlace=function(placeHandle,shift,verticalPos,startConf,noVerticalArmForUpMovement)
    local m,angle=getBoxAdjustedMatrixAndFacingAngle(placeHandle)
    m[4]=m[4]+m[2]*shift
    m[8]=m[8]+m[6]*shift
    sim.setObjectPosition(vehicleTarget,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
    sim.setObjectOrientation(vehicleTarget,{0,0,angle})
    local params = {
        joints = armJoints,
        targetPos = startConf,
        maxVel = fkSpeed,
        maxAccel = fkAccel,
        maxJerk = fkJerk,
    }
    sim.moveToConfig(params)
    
    waitToReachVehicleTargetPositionAndOrientation()
    setGripperTargetFixedToWorld()
    local q=sim.getObjectPose(gripperTip)
    q[1]=m[4]
    q[2]=m[8]
    local ikparams = {
        pose = sim.getObjectPose(gripperTip),
        targetPose = q,
        maxVel = ikSpeed,
        maxAccel = ikAccel,
        maxJerk = ikJerk,
        callback = ikcb2,
        auxData = ikData,
        metric = ikMetric
    }
    sim.moveToPose(ikparams)
    q[3]=verticalPos
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)
    openGripper()
    q[3]=q[3]+0.1
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)
    setGripperTargetMovingWithVehicle()    
end

function ikcb1(data)
    sim.setObjectPose(data.auxData.target,data.pose)
    simIK.handleGroup(data.auxData.ikEnv,data.auxData.ikGroups[1],{syncWorlds=true})
end

function ikcb2(data)
    sim.setObjectPose(data.auxData.target,data.pose)
    simIK.handleGroup(data.auxData.ikEnv,data.auxData.ikGroups[2],{syncWorlds=true})
end

function sysCall_thread()
    local base=sim.getObject('..')
    gripperTarget=sim.getObject('../youBot_gripperPositionTarget')
    gripperTip=sim.getObject('../youBot_gripperPositionTip')
    local base2=sim.getObject('../Rectangle22')
    local constrBase2=sim.getObject('../youBot_gripperOrientationTarget')
    local gripperTarget2=sim.getObject('../youBot_gripperOrientationTarget')
    local gripperTip2=sim.getObject('../youBot_gripperOrientationTip')
    
    redBox1=sim.getObject('/redRectangle1')
    yellowBox1=sim.getObject('/yellowRectangle1')
    yellowBox2=sim.getObject('/yellowRectangle2')
    greenBox1=sim.getObject('/greenRectangle1')
    greenBox2=sim.getObject('/greenRectangle2')
    greenBox3=sim.getObject('/greenRectangle3')
    armJoints={-1,-1,-1,-1,-1}
    for i=0,4,1 do
        armJoints[i+1]=sim.getObject('../youBotArmJoint'..i)
    end
    gripper=sim.getObject('../youBot_gripper')
    place1=sim.getObject('/place1')
    place2=sim.getObject('/place2')
    place3=sim.getObject('/place3')

    pickup1={0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,0*math.pi/180}
    pickup2={0,-13.39*math.pi/180,-93.91*math.pi/180,-72.72*math.pi/180,90*math.pi/180}
    pickup3={0,-14.52*math.pi/180,-70.27*math.pi/180,-95.27*math.pi/180,90*math.pi/180}
    platformIntermediateDrop={0,16*math.pi/180,52*math.pi/180,73*math.pi/180,0*math.pi/180}
    platformDrop1={0,54.33*math.pi/180,32.88*math.pi/180,35.76*math.pi/180,0*math.pi/180}--{0,-0.4,0.2}
    platformDrop2={0,40.74*math.pi/180,45.81*math.pi/180,59.24*math.pi/180,0*math.pi/180}--{0,-0.32,0.2}
    platformDrop3={0,28.47*math.pi/180,55.09*math.pi/180,78.32*math.pi/180,0*math.pi/180}--{0,-0.24,0.2}

    dist1=0.2
    dropHeight1=0.035
    dropHeight2=0.095
    dropHeight3=0.155
    ikSpeed={0.2}
    ikAccel={0.1}
    ikJerk={0.1}
    fkSpeed={1,1,1,1,1}
    fkAccel={0.6,0.6,0.6,0.6,0.6}
    fkJerk={1,1,1,1,1}
    ikMetric={1,1,1,0.1}
    

    ikEnv=simIK.createEnvironment()
    
    -- Prepare the ik groups, using the convenience function 'simIK.addElementFromScene':
    ikGroup1=simIK.createGroup(ikEnv)
    simIK.addElementFromScene(ikEnv,ikGroup1,base,gripperTip,gripperTarget,simIK.constraint_position)

    ikGroup2=simIK.createGroup(ikEnv)
    simIK.setGroupCalculation(ikEnv,ikGroup2,simIK.method_damped_least_squares,0.01,100)
    simIK.addElementFromScene(ikEnv,ikGroup2,base,gripperTip,gripperTarget,simIK.constraint_position)
    local ikElement,simToIkMap=simIK.addElementFromScene(ikEnv,ikGroup2,base2,gripperTip2,gripperTarget2,simIK.constraint_gamma)
    simIK.setElementBase(ikEnv,ikGroup2,ikElement,simToIkMap[base2],simToIkMap[constrBase2])

    ikData={}
    ikData.ikEnv=ikEnv
    ikData.ikGroups={ikGroup1,ikGroup2}
    ikData.target=gripperTarget

    setGripperTargetMovingWithVehicle()
    openGripper()


    -- redBox first pickup:
    m,angle=getBoxAdjustedMatrixAndFacingAngle(redBox1)
    sim.setObjectPosition(vehicleTarget,{m[4]-m[1]*dist1,m[8]-m[5]*dist1,0})
    sim.setObjectOrientation(vehicleTarget,{0,0,angle})
    sim.setFloatSignal('cameraJoint',-159*math.pi/180)
    waitToReachVehicleTargetPositionAndOrientation()
    local params = {
        joints = armJoints,
        targetPos = pickup1,
        maxVel = fkSpeed,
        maxAccel = fkAccel,
        maxJerk = fkJerk,
    }
    sim.moveToConfig(params)
    local q=sim.getObjectPose(gripperTip)
    q[1]=m[4]
    q[2]=m[8]
    q[3]=m[12]
    local ikparams = {
        pose = sim.getObjectPose(gripperTip),
        targetPose = q,
        maxVel = ikSpeed,
        maxAccel = ikAccel,
        maxJerk = ikJerk,
        callback = ikcb2,
        auxData = ikData,
        metric = ikMetric
    }
    sim.moveToPose(ikparams)
    closeGripper()
    q[3]=q[3]+0.05
    ikparams.pose = sim.getObjectPose(gripperTip)
    ikparams.targetPose = q
    sim.moveToPose(ikparams)

    -- redBox first drop:
    dropToPlace(place3,0,dropHeight1,pickup2,false)
    -- yellow box1 first pickup and intermediate drop:
    pickupBoxFromPlace(yellowBox1,pickup2)
    -- yellow box1 intermediate drop onto platform:
    dropToPlatform(platformDrop2)
    sim.setFloatSignal('cameraJoint',20*math.pi/180)

    --yellow box2 first pickup:
    pickupBoxFromPlace(yellowBox2,pickup2)
    -- yellow box2 first drop:
    dropToPlace(place2,0.04,dropHeight1,pickup2,false)
    pickupFromPlatformAndReorient(yellowBox1)
    dropToPlace(place2,-0.04,dropHeight1,pickup2,false)


    pickupBoxFromPlace(redBox1,pickup2)
    dropToPlace(place2,0,dropHeight2,pickup2,false)

    sim.setFloatSignal('cameraJoint',-50*math.pi/180)

    pickupBoxFromPlace(greenBox1,pickup2)
    sim.setInt32Param(sim.intparam_current_page,4)
    dropToPlatform(platformDrop1)
    pickupBoxFromPlace(greenBox2,pickup2)
    dropToPlatform(platformDrop3)
    pickupBoxFromPlace(greenBox3,pickup2)
    sim.setFloatSignal('cameraJoint',80*math.pi/180)
    dropToPlace(place3,0.08,dropHeight1,pickup2,false)
    sim.setInt32Param(sim.intparam_current_page,0)
    pickupFromPlatformAndReorient(greenBox2)
    dropToPlace(place3,0.0,dropHeight1,pickup2,false)
    pickupFromPlatformAndReorient(greenBox1)
    dropToPlace(place3,-0.08,dropHeight1,pickup2,false)


    pickupBoxFromPlace(redBox1,pickup2)
    dropToPlace(place1,0,dropHeight1,pickup2,false)


    pickupBoxFromPlace(yellowBox1,pickup2)
    dropToPlatform(platformDrop2)
    pickupBoxFromPlace(yellowBox2,pickup2)
    dropToPlace(place3,0.04,dropHeight2,pickup2,false)
    pickupFromPlatformAndReorient(yellowBox1)

    local params = {
        joints = armJoints,
        targetPos = pickup1,
        maxVel = fkSpeed,
        maxAccel = fkAccel,
        maxJerk = fkJerk,
    }
    params.pos = getConfig(armJoints)
    params.targetPos = pickup3
    sim.moveToConfig(params)
    dropToPlace(place3,-0.04,dropHeight2,pickup3,true)


    pickupBoxFromPlace(redBox1,pickup1)


    dropToPlace(place3,0,dropHeight3,pickup1,true)

    sim.setObjectPosition(vehicleTarget,{0,0,0})
    sim.wait(2)
    sim.setObjectOrientation(vehicleTarget,{0,0,0})
    params.pos = getConfig(armJoints)
    params.targetPos = platformIntermediateDrop
    sim.moveToConfig(params)
    waitToReachVehicleTargetPositionAndOrientation()
    sim.stopSimulation()
end

function sysCall_cleanup()
    simIK.eraseEnvironment(ikEnv)
end