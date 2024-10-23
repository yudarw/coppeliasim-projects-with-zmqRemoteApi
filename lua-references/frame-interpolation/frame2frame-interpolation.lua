--lua

sim=require'sim'

function sysCall_thread()
    -- You can use 2 methods to interpolate between two reference frames:
    frame1=sim.getObject('/ReferenceFrame1')
    frame2=sim.getObject('/ReferenceFrame2')
    matrix1=sim.getObjectMatrix(frame1)
    matrix2=sim.getObjectMatrix(frame2)
    useMethodNb=2

    if useMethodNb==1 then
        -- Method 1
        rotAxis,angle=sim.getRotationAxis(matrix1,matrix2)
        pos1=sim.getObjectPosition(frame1)
        pos2=sim.getObjectPosition(frame2)
        posVector={pos2[1]-pos1[1],pos2[2]-pos1[2],pos2[3]-pos1[3]}

        -- Now let's have frame1 move to frame2 in 200 steps:
        stepCnt=1000
        dAngle=angle/stepCnt
        dPos={posVector[1]/stepCnt,posVector[2]/stepCnt,posVector[3]/stepCnt}
        for i=1,stepCnt,1 do
            -- Take care of the orientation:
            matrixOut=sim.rotateAroundAxis(matrix1,rotAxis,{0,0,0},dAngle*i)
            -- Take care of the position:
            matrixOut[4]=pos1[1]+dPos[1]*i
            matrixOut[8]=pos1[2]+dPos[2]*i
            matrixOut[12]=pos1[3]+dPos[3]*i
            -- Apply the new matrix to frame1:
            sim.setObjectMatrix(frame1,matrixOut)
            -- Wait for next simulation step:
            sim.step()
        end
    else
        -- Method 2
        -- Now let's have frame1 move to frame2 in 200 steps:
        stepCnt=10
        for i=1,stepCnt,1 do
            matrixOut=sim.interpolateMatrices(matrix1,matrix2,i/stepCnt)
            -- Apply the new matrix to frame1:
            sim.setObjectMatrix(frame1,matrixOut)
            -- Wait for next simulation step:
            sim.step()
        end
    end
end