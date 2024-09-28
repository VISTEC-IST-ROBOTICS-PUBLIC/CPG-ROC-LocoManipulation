--[[  
Function for handling slider changes on the UI  
--]]
function sliderChange(ui,id,newVal)
    if id == 10+0 then
        simUI.setLabelText(ui,1000+10,'Value: '..newVal)
        tuneParameters[1] = newVal
    elseif id == 10+1 then
        simUI.setLabelText(ui,1000+11,'Value: '..newVal)
        tuneParameters[2] = newVal
    elseif id == 10+2 then
        simUI.setLabelText(ui,1000+12,'Value: '..newVal)
        tuneParameters[3] = newVal
    elseif id == 20+0 then
        simUI.setLabelText(ui,1000+20,'Value: '..newVal)
        tuneParameters[4] = newVal
    elseif id == 20+1 then
        simUI.setLabelText(ui,1000+21,'Value: '..newVal)
        tuneParameters[5] = newVal
    elseif id == 20+2 then
        simUI.setLabelText(ui,1000+22,'Value: '..newVal)
        tuneParameters[6] = newVal
    elseif id == 30+0 then
        simUI.setLabelText(ui,1000+30,'Value: '..newVal)
        tuneParameters[7] = newVal
    elseif id == 30+1 then
        simUI.setLabelText(ui,1000+31,'Value: '..newVal)
        tuneParameters[8] = newVal
    elseif id == 30+2 then
        simUI.setLabelText(ui,1000+32,'Value: '..newVal)
        tuneParameters[9] = newVal    
    elseif id == 40+0 then
            simUI.setLabelText(ui,1000+40,'Value: '..newVal)
            tuneParameters[10] = newVal
    elseif id == 50+1 then
        simUI.setLabelText(ui,1000+51,'Value: '..newVal/10)
        tuneParameters[12] = newVal/10    
    elseif id == 50+2 then
        simUI.setLabelText(ui,1000+52,'Value: '..newVal/100)
        tuneParameters[13] = newVal/100
    elseif id == 50+3 then
        simUI.setLabelText(ui,1000+53,'Value: '..newVal/10)
        tuneParameters[14] = newVal/10
    elseif id == 50+4 then
        simUI.setLabelText(ui,1000+54,'Value: '..newVal/100)
        tuneParameters[24] = newVal/100
    elseif id == 60+1 then
        simUI.setLabelText(ui,1000+61,'Value: '..newVal/10)
        tuneParameters[15] = newVal/10
    elseif id == 60+2 then
        simUI.setLabelText(ui,1000+62,'Value: '..newVal/10)
        tuneParameters[16] = newVal/10
    elseif id == 70+1 then
        simUI.setLabelText(ui,1000+71,'Value: '..newVal)
        tuneParameters[17] = newVal  
    elseif id == 70+2 then
        simUI.setLabelText(ui,1000+72,'Value: '..newVal/100)
        tuneParameters[18] = newVal/100
    elseif id == 70+3 then
        simUI.setLabelText(ui,1000+73,'Value: '..newVal/10)
        tuneParameters[19] = newVal/10
    elseif id == 80+1 then
        simUI.setLabelText(ui,1000+81,'Value: '..newVal/100)
        tuneParameters[21] = newVal/100    
    elseif id == 40+1 then
        simUI.setLabelText(ui,1000+41,'Value: '..newVal/100)
        tuneParameters[22] = newVal/100 
    elseif id == 40+2 then
        simUI.setLabelText(ui,1000+42,'Value: '..newVal/100)
        tuneParameters[23] = newVal/100    
    elseif id == 90+1 then
        simUI.setLabelText(ui,1000+91,'Value: '..newVal)
        tuneParameters[25] = newVal 
    -- elseif id == 90+2 then
    --     simUI.setLabelText(ui,1000+92,'Value: '..newVal)
    --     tuneParameters[26] = newVal    
    -- elseif id == 90+3 then
    --     simUI.setLabelText(ui,1000+93,'Value: '..newVal)
    --     tuneParameters[27] = newVal 
    end
end

function checkboxChange(ui,id,newVal)
    if id == 50 then
        if simUI.getCheckboxValue(ui,id) > 0 then
            tuneParameters[11] = newVal
        else
            tuneParameters[11] = newVal
        end
    elseif id == 80 then
        if simUI.getCheckboxValue(ui,id) > 0 then
            tuneParameters[20] = newVal
        else
            tuneParameters[20] = newVal
        end

    end
    print(tuneParameters[20])
end

-- Test parameter alpha
-- function sliderChange(ui, id, newVal)
--     if id == 10+0 then
--         simUI.setLabelText(ui,1000+0,'Value: '..newVal/10)
--         tuneParameters[1] = newVal/10
-- --     -- elseif id == 10+1 then
-- --     --     simUI.setLabelText(ui,1000+1,'Value: '..newVal)
-- --     --     testParameters[2] = newVal
--     end
-- end

function sysCall_init()
    -- do some initialization here
    --simRemoteApi.start(19999)
    --res = sim.launchExecutable('../workspace/neurobpy/projects/cpg_balancing/catkin_ws/src/cpg_balancing/scripts/main.py','', 1)
    --print(res)
    sim_counter = 0
    degToRad = 22/7/180
    -- configuration parameters
    print("----------Setting Up Simulation-----------")
    leg_num = 6
    legJoint_num = 3
    getJointstate = false
    dataPlot = {}
    -- system activation
    activate_vision = false
    activate_rollControl = true

    -- Experiment data system
    -- j_angle_file=io.open('./Experiment_data/rolling/Exp_data_rolling_sim.txt','w+')
    -- j_angle_file:write('Joint angles for each simulation step:\n\n')
    
    -- robotHandle = sim.getObject("laikago")
    -- Handle
    robotHandle = sim.getObject(".")
    -- ballHandle = sim.getObject("Sphere_C")    
    ball_pos = {}
    motor_string = {"./Abdomen/BC", "./Abdomen/CF", "./Abdomen/FT"}
    jointHandle = {}
    jointPos = {}
    jointVel = {}
    jointTorque = {}
    jointPosTarget = {}
    footContactHandle = {}
    footContact = {}
    fc_force = {{},{},{},{},{},{}}
    fc_torque = {{},{},{},{},{},{}}
    fc_force_o = {{},{},{},{},{},{}}
    fc_torque_o = {{},{},{},{},{},{}}
    fc_dataStream = {}
    leg_color = {{1,0,0},{1,1,0},{0,1,0},{1,0,1},{0,1,1},{1,0.8,0}}
    -- orient = {}
    tranMatrix = {}
    vector_w = {}
    -- sphereBaseHandle = sim.getObject("Sphere_base")
    local joint_count = 1
    for i=1, leg_num*legJoint_num + 1 do
        jointHandle[i] = 0 -- joint Handle array
        jointPos[i] = 0 -- target position array
        jointVel[i] = 0 -- target position array
        jointPosTarget[i] = 0 -- target position array
        jointTorque[i] = 0 -- target position array
    end
    for i = 0, legJoint_num-1, 1 do
        for j = 0, leg_num-1, 1 do
            jointHandle[joint_count] = sim.getObject(tostring(motor_string[i+1] .. tostring(j)))
            joint_count = joint_count+1
        end
    end
    jointHandle[joint_count] = sim.getObject("./Abdomen/TA")

    -- FootContact Force Visualization
    footContact_graph = sim.getObject("/FC_graph")
    for i=1, leg_num do
        footContactHandle[i] = sim.getObject(tostring("/Force_sensor" .. tostring(i-1))) -- footContact Handle
        footContact[i] = 0 -- footContact array
        fc_dataStream[i] = sim.addGraphStream(footContact_graph,tostring("./fc_leg" .. tostring(i-1)),'N',0,leg_color[i])
        fc_force_o[i] = {0,0,0}
        fc_torque_o[i] = {0,0,0}
        -- orient[i] = {}
        tranMatrix[i] = {}
        vector_w[i] = {}    
    end
    print(jointHandle)

    -- jointPos_graph = sim.getObject("./jointPos_graph")
    -- orientation_graph = sim.getObject("./orientation_graph")
    -- ballpos_graph = sim.getObject("./ballpos_graph")
    -- ballposx=sim.addGraphStream(ballpos_graph,'ball_x','m',0,{1,0,0})
    -- ballposy=sim.addGraphStream(ballpos_graph,'ball_y','m',0,{0,1,0})
    -- leg0Path_graph=sim.getObject("./leg0_path")
    -- leg0Trace=sim.addDrawingObject(sim.drawing_linestrip+sim.drawing_cyclic,2,0,-1,200,{1,1,0},nil,nil,{1,1,0})
    -- trackingErrBC_graph = sim.getObject("tracking_errorBC_graph")
    -- jointErrBC1=sim.addGraphStream(trackingErrBC_graph,'pos_errorBC1','deg',0,{1,0,0})
    -- jointErrBC2=sim.addGraphStream(trackingErrBC_graph,'pos_errorBC2','deg',0,{0,1,0})
    -- jointErrBC3=sim.addGraphStream(trackingErrBC_graph,'pos_errorBC3','deg',0,{0,1,1})
    -- trackingErrCF_graph = sim.getObject("tracking_errorCF_graph")
    -- jointErrCF1=sim.addGraphStream(trackingErrCF_graph,'pos_errorCF1','deg',0,{1,0,0})
    -- jointErrCF2=sim.addGraphStream(trackingErrCF_graph,'pos_errorCF2','deg',0,{0,1,0})
    -- jointErrCF3=sim.addGraphStream(trackingErrCF_graph,'pos_errorCF3','deg',0,{0,1,1})
    -- jointTorqueBC_graph = sim.getObject("jointTorqueBC_graph")
    -- jointTorqBC1=sim.addGraphStream(jointTorqueBC_graph,'BC0','deg',0,{1,0,0})
    -- jointTorqBC2=sim.addGraphStream(jointTorqueBC_graph,'BC1','deg',0,{0,1,0})
    -- jointTorqBC3=sim.addGraphStream(jointTorqueBC_graph,'BC2','deg',0,{0,1,1})
    -- jointTorqueCF_graph = sim.getObject("jointTorqueCF_graph")
    -- jointTorqCF1=sim.addGraphStream(jointTorqueCF_graph,'CF0','deg',0,{1,0,0})
    -- jointTorqCF2=sim.addGraphStream(jointTorqueCF_graph,'CF1','deg',0,{0,1,0})
    -- jointTorqCF3=sim.addGraphStream(jointTorqueCF_graph,'CF2','deg',0,{0,1,1})
    
    -- ROS node
    if simROS2 then
        -- Publisher
        jointPosPub=simROS2.createPublisher('/jointPosition', 'std_msgs/msg/Float32MultiArray')
        jointVelPub=simROS2.createPublisher('/jointVelocity', 'std_msgs/msg/Float32MultiArray')
        robotPosPub=simROS2.createPublisher('/robotPosition', 'std_msgs/msg/Float32MultiArray')
        jointTorqPub=simROS2.createPublisher('/jointTorque', 'std_msgs/msg/Float32MultiArray')
        footContactPub=simROS2.createPublisher('/footContact', 'std_msgs/msg/Float32MultiArray')
        imuPub=simROS2.createPublisher('/imu', 'std_msgs/msg/Float32MultiArray')
        tuneParamPub=simROS2.createPublisher('/tuneParameter', 'std_msgs/msg/Float32MultiArray')
        terminateControllerPub=simROS2.createPublisher('/terminateController', 'std_msgs/msg/Bool')
        -- -- triggerEpPub=simROS2.createPublisher('/triggerEpisode', 'std_msgs/Bool')
        -- -- Subscriber
        motorComSub=simROS2.createSubscription('/motorCommand', 'std_msgs/msg/Float32MultiArray', 'motorCommand_callback')
        motorMaxTorqueComSub=simROS2.createSubscription('/motorMaxTorqueCommand', 'std_msgs/msg/Float32MultiArray', 'motorMaxTorqueCommand_callback')
        dataPlotSub=simROS2.createSubscription('/dataPlot', 'std_msgs/msg/Float32MultiArray', 'dataPlot_callback')
    else
        sim.addLog(sim.verbosity_scripterrors,"ROS interface was not found. Child script.")
    end
    
    -- IMU
    -- Gyro
    gyroCommunicationTube=sim.tubeOpen(0,'gyroData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase
    roll = 0
    pitch = 0
    yaw = 0
    r_w = 0.5
    i_w = 0.5
    roll_filter = 0
    pitch_filter = 0
    -- -- Accelerometer
    accelCommunicationTube=sim.tubeOpen(0,'accelerometerData'..sim.getNameSuffix(nil),1) -- put this in the initialization phase

    -- User Interface setup
    xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Neutron UI" on-close="closeEventHandler" >
    <label text="This is a demo of the CustomUI plugin for Legged Robot Platform Brain VISTEC." wordwrap="true" />
    <tabs>
        <tab title="Joint Angle" layout="grid">
            <label text="BCFront" wordwrap="true" />
            <label text="Value: 0" id="1010" wordwrap="true" />
            <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
            <label text="BCMiddle" wordwrap="true" />
            <label text="Value: 0" id="1011" wordwrap="true" />
            <hslider id="11" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
            <label text="BCHind" wordwrap="true" />
            <label text="Value: 0" id="1012" wordwrap="true" />
            <hslider id="12" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
            <br />
            <label text="CFFront" wordwrap="true" />
            <label text="Value: 0" id="1020" wordwrap="true" />
            <hslider id="20" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
            <label text="CFMiddle" wordwrap="true" />
            <label text="Value: 0" id="1021" wordwrap="true" />
            <hslider id="21" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
            <label text="CFHind" wordwrap="true" />
            <label text="Value: 0" id="1022" wordwrap="true" />
            <hslider id="22" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
            <br />

            <label text="FTFront" wordwrap="true" />
            <label text="Value: 0" id="1030" wordwrap="true" />
            <hslider id="30" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
            <label text="FTMiddle" wordwrap="true" />
            <label text="Value: 0" id="1031" wordwrap="true" />
            <hslider id="31" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
        
            <label text="FTHind" wordwrap="true" />
            <label text="Value: 0" id="1032" wordwrap="true" />
            <hslider id="32" tick-position="both-sides" tick-interval="1" minimum="-90" maximum="90" on-change="sliderChange" style="plastique" />
            <br />
        
            <label text="TA_joint" wordwrap="true" />
            <label text="Value: 0" id="1040" wordwrap="true" />
            <hslider id="40" tick-position="both-sides" tick-interval="1" minimum="-10" maximum="35" on-change="sliderChange" style="plastique" />

            <label text="Front_MI" wordwrap="true" />
            <label text="Value: 0" id="1041" wordwrap="true" />
            <hslider id="41" tick-position="both-sides" tick-interval="1" minimum="-1" maximum="20" on-change="sliderChange" style="plastique" />

            <label text="back_MI" wordwrap="true" />
            <label text="Value: 0" id="1042" wordwrap="true" />
            <hslider id="42" tick-position="both-sides" tick-interval="1" minimum="-1" maximum="20" on-change="sliderChange" style="plastique" />

        </tab>
        <tab title="Local-Leg" layout="vbox">
            <checkbox text="Ground Searching Control" on-change="checkboxChange" id="50" />
            <group layout="grid">
                <label text="fc_threshold" wordwrap="true" />
                <label text="Value: 0" id="1051" wordwrap="true" />
                <hslider id="51" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
                <br/>
                <label text="fc_selfw" wordwrap="true" />
                <label text="Value: 0" id="1052" wordwrap="true" />
                <hslider id="52" tick-position="both-sides" tick-interval="1" minimum="0" maximum="100" on-change="sliderChange" style="plastique" />
                <br/>
                <label text="fc_err_fac" wordwrap="true" />
                <label text="Value: 0" id="1053" wordwrap="true" />
                <hslider id="53" tick-position="both-sides" tick-interval="1" minimum="0" maximum="10" on-change="sliderChange" style="plastique" />
                <br/>
                <label text="ball distance ref" wordwrap="true" />
                <label text="Value: 0" id="1054" wordwrap="true" />
                <hslider id="54" tick-position="both-sides" tick-interval="1" minimum="-100" maximum="0" on-change="sliderChange" style="plastique" />
            </group>
        </tab>   
        <tab title="Decouple CPG" layout="vbox">
            <group layout="grid">
                <label text="cpg_input_w_front" wordwrap="true" />
                <label text="Value: 0" id="1061" wordwrap="true" />
                <hslider id="61" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
                <br/>
                <label text="cpg_input_w_back" wordwrap="true" />
                <label text="Value: 0" id="1062" wordwrap="true" />
                <hslider id="62" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
            </group>
        </tab>          
        <tab title="Orientation Control" layout="vbox">
            <group>
                <label text="Pitch Control." wordwrap="true" />
                <label text="pitch_bias" wordwrap="true" />
                <label text="Value: 0" id="1071" wordwrap="true" />
                <hslider id="71" tick-position="both-sides" tick-interval="1" minimum="-20" maximum="20" on-change="sliderChange" style="plastique" />
                <label text="pitch_freq_mod_fac" wordwrap="true" />
                <label text="Value: 0" id="1072" wordwrap="true" />
                <hslider id="72" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
                <label text="pitch_jointBias_mod_fac" wordwrap="true" />
                <label text="Value: 0" id="1073" wordwrap="true" />
                <hslider id="73" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
            </group>
            <group>
            <checkbox text="activate_tilting_stance_reflex" on-change="checkboxChange" id="80" />
                <label text="Roll Control." wordwrap="true" />
                <label text="roll_leg_fac" wordwrap="true" />
                <label text="Value: 0" id="1081" wordwrap="true" />
                <hslider id="81" tick-position="both-sides" tick-interval="1" minimum="0" maximum="100" on-change="sliderChange" style="plastique" />
            </group>
        </tab>
        <tab title="Neuromechanical Control" layout="vbox">
            <group>
                <label text="front max torque" wordwrap="true" />
                <label text="Value: 0" id="1091" wordwrap="true" />
                <hslider id="91" tick-position="both-sides" tick-interval="1" minimum="0" maximum="200" on-change="sliderChange" style="plastique" />
            </group>
        </tab>
    
    </tabs>
    </ui> ]]
    -- -- User Interface setup
    -- <tab title="Image">
    -- <image id="5007" keep-aspect-ratio="true" scaled-contents="true" style="* {background-color: #555555}" />
    -- </tab>  
    -- xml = [[ <ui closeable="false" resizable="true" style="plastique" title="Cpg_balancing">
    -- <label text="alpha" wordwrap="true" />
    -- <label text="Value: 0" id="1000" wordwrap="true" />
    -- <hslider id="10" tick-position="both-sides" tick-interval="1" minimum="0" maximum="50" on-change="sliderChange" style="plastique" />
    -- <tab title="Muscle Model" layout="vbox">
    --     <group>
    --         <label text="a" wordwrap="true" />
    --         <label text="Value: 0" id="1091" wordwrap="true" />
    --         <hslider id="91" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
    --         <label text="b" wordwrap="true" />
    --         <label text="Value: 0" id="1092" wordwrap="true" />
    --         <hslider id="92" tick-position="both-sides" tick-interval="1" minimum="0" maximum="50" on-change="sliderChange" style="plastique" />
    --         <label text="gamma" wordwrap="true" />
    --         <label text="Value: 0" id="1093" wordwrap="true" />
    --         <hslider id="93" tick-position="both-sides" tick-interval="1" minimum="0" maximum="20" on-change="sliderChange" style="plastique" />
    --     </group>
    -- </tab>

    -- </ui> ]]
    -- Create User Interface
    ui=simUI.create(xml)
    x, y=simUI.getPosition(ui)
    simUI.setPosition(ui, x+500, y-250, true)
    print("...")
    -- initializa slider values
    -- tuneParameters = {5.0} -- value = x/10
    tuneParameters = {10,   10, 0,  -75, -35, -35,   10, -5, 0,  25, 0.1, 0.3}		--walking JointBias V1
    -- tuneParameters = {10,   10, 0,  -75, -35, -35,   10, -5, 0,  25, 1, 0}		--Rolling JointBias V1
    -- tuneParameters = {10,   10, 0,  -75, -30, -30,   10, -5, 0,  25, 1, 0}		--Rolling Test MEMO V1

    -- Big Ball 
    -- tuneParameters = {   10,   0, -10,          --BC         --Rolling JointBias Big Ball
    --                     -75, -20, -60,      --CF
    --                      10,  20,    0,       --FT
    --                     15,                 --TA
    --                     2, 5, 96, 2,		-- GS control (activation/10, force threshold/100, recurrent factor/10, push factor/10)
    --                     2, 2,               -- decouple cpg (front leg cpg input fac/10, middle, hind leg cpg input fac/10)
    --                     -10, 0.5, 0.05,      -- pitch control (pitch_bias,freq_mod/100, jointBias_mod/10)
    --                     2, 30,              -- Roll Control (activate[0,2], roll_leg_fac[/10])
    --                     12, 6,              -- CPG Frequency (front_MI/10, back_MI/10)
    --                     0,
    --                     7                }  -- neuromechanical 

    --                     25,                 --TA
    --                     2, 5, 96, 2,		-- GS control (activation/10, force threshold/100, recurrent factor/10, push factor/10)
    --                     3, 0,               -- decouple cpg (front leg cpg input fac/10, middle, hind leg cpg input fac/10)
    --                     0, 0.4, 0.05,      -- pitch control
    --                     2, 30, 
    --                     3, 3,
    --                     0,
    --                     4                }  -- neuromechanical 

    -- Small Ball 
    -- tuneParameters = {   10,   0, 0,        --BC (front, middle Hind leg) Rolling JointBias Big Ball
    --                     -75, -15, -50,      --CF 
    --                      10,  10,  -20,     --FT 10,  10,  -20
    --                     15,                 --TA
    --                     2, 5, 96, 2,		-- GS control (activation/10, force threshold/100, recurrent factor/10, push factor/10)
    --                     2, 2,               -- decouple cpg (front leg cpg input fac/10, middle, hind leg cpg input fac/10)
    --                     0, 0.5, 0.05,      -- pitch control (pitch_bias,freq_mod/100, jointBias_mod/10)
    --                     2, 30,              -- Roll Control (activate[0,2], roll_leg_fac[/10])
    --                     12, 6,              -- CPG Frequency (front_MI/10, back_MI/10)
    --                     -50,
    --                     7                }  -- neuromechanical 

    -- locomotion 
    tuneParameters = {   10,   0, 0,        --BC (front, middle Hind leg) Rolling JointBias Big Ball
                        -75, -35, -35,      --CF 
                        10, -5, 0,     --FT 10,  10,  -20
                        25,                 --TA
                        2, 5, 96, 2,		-- GS control (activation/10, force threshold/100, recurrent factor/10, push factor/10)
                        2, 2,               -- decouple cpg (front leg cpg input fac/10, middle, hind leg cpg input fac/10)
                        -10, 0.5, 0.05,      -- pitch control (pitch_bias,freq_mod/100, jointBias_mod/10)
                        0, 30,              -- Roll Control (activate[0,2], roll_leg_fac[/10])
                        12, 6,              -- CPG Frequency (front_MI/10, back_MI/10)
                        -50,
                        7                }  -- neuromechanical 
    sliderID = {10,11,12, 
                20,21,22, 
                30,31,32, 
                40, 
                50,51,52,53, 
                61,62, 
                71,72,73, 
                80,81, 
                41,42, 
                54, 
                91}

    checkboxID = {50}
    for i=1, table.getn(sliderID) do
        if sliderID[i] == 50 then
            simUI.setCheckboxValue(ui, sliderID[i], tuneParameters[i])
            checkboxChange(ui, sliderID[i], tuneParameters[i])
        elseif sliderID[i] == 80 then
            simUI.setCheckboxValue(ui, sliderID[i], tuneParameters[i])
            checkboxChange(ui, sliderID[i], tuneParameters[i])
        else
            simUI.setSliderValue(ui, sliderID[i], tuneParameters[i])
            sliderChange(ui, sliderID[i], tuneParameters[i])
        end
    end


    -- Start the client application (c++ node)
    result=sim.launchExecutable('../workspace/gorobots/projects/db_alpha/walking_neural_control/sim/ros2_ws/src/walking_neural_controller_sim/bin/db_beta_controller_sim', 'test',0)
    -- print(result)
    if (result==false) then
        sim.displayDialog('Error','External ROS-Node not found',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end
    
    print("----------Finished Setting Up -----------")

end

function motorCommand_callback(msg)
    jointPosTarget = msg.data
    -- if getJointstate == true then 
    --     sim.setObjectInt32Parameter(robotHandle, 3003, 1)    
    -- elseif getJointstate == false then
    --     getJointstate = true    
    for i = 1, legJoint_num*leg_num+1 do 
        sim.setJointTargetPosition(jointHandle[i], jointPosTarget[i])
    end        
    --     sim.setObjectInt32Parameter(robotHandle, 3003, 0)
    -- end
end
function motorMaxTorqueCommand_callback(msg)
    jointMaxTorque = msg.data
    -- print(jointMaxTorque)
    local count = 1
    for i = 1, 18, 3 do
        sim.setJointMaxForce(jointHandle[i], jointMaxTorque[count])
        -- print(jointHandle[i], jointMaxTorque[count])
        count = count + 1
    end
end

function dataPlot_callback(msg)
    dataPlot = msg.data
    -- sim.setGraphUserData(jointPos_graph, "fcphase", dataPlot[1])

end

function resetModel_callback(msg)

end

function sysCall_actuation()
    -- put your actuation code here
    
end

function sysCall_sensing()
    -- put your sensing code here
    
    -- joint Position Feedback
    for i = 1, 19 do 
        jointPos[i] = sim.getJointPosition(jointHandle[i])
        -- jointVel[i] = sim.getJointVelocity(jointHandle[i])
        jointTorque[i] = sim.getJointForce(jointHandle[i])
    end
    -- print(jointTorque[7])
    for i = 1, 6 do 
        -- read force sensors value and publish force sensor value
        result, fc_force[i], fc_torque[i] = simReadForceSensor(footContactHandle[i])
        for j = 1, 3 do 
            fc_force[i][j] = 0.3 * fc_force[i][j] + 0.7 * fc_force_o[i][j]
            fc_torque[i][j] = 0.3 * fc_torque[i][j] + 0.7 * fc_torque_o[i][j]
        end
        --footContact[i] = math.sqrt(fc_force[1]^2+fc_force[2]^2+fc_force[3]^2)
        -- sim.setGraphStreamValue(footContact_graph,fc_dataStream[i],fc_force[i][1])
        -- sim.setGraphStreamValue(footContact_graph,fc_dataStream[i],fc_torque[i][1])
    end
    fc_force_o = fc_force
    fc_torque_o = fc_torque    
    -- sim.setGraphStreamValue(footContact_graph,fc_dataStream[1],fc_force[3])

    -- -- print(footContact)
                
    -- -- IMU
    -- gyroData=sim.tubeRead(gyroCommunicationTube)
    -- accelData=sim.tubeRead(accelCommunicationTube)

    -- if (gyroData and accelData) then
    --     imu_data=sim.unpackFloatTable(gyroData) -- (omegax,y,z, roll, pitch raw)
    --     acceleration=sim.unpackFloatTable(accelData)
        
    --     omegax   = imu_data[1]*180*7/22
    --     omegay   = imu_data[2]*180*7/22
    --     omegaz   = imu_data[3]*180*7/22
    --     roll     = imu_data[4]*180*7/22
    --     pitch    = imu_data[5]*180*7/22
    --     yaw      = imu_data[6]*180*7/22
    --     inclinationSensors={roll, pitch, yaw}
    --     roll_filter  = roll*i_w  + roll_filter*r_w
    --     pitch_filter = pitch*i_w + pitch_filter*r_w
    --     -- print(imu_data)
    --     imu = {omegax, omegay, omegaz, roll, pitch, yaw, acceleration[1], acceleration[2], acceleration[3]}
    --     -- print(imu)
    -- end    

    -- -- Vision sensor
    -- if activate_vision then
    --     s=sim.getObject("Vision_sensor")
    --     img,x,y=sim.getVisionSensorCharImage(s)
    --     simUI.setImageData(ui,5007,img,x,y)
    -- end

    -- publish data
    -- print(tuneParameters)
    simROS2.publish(jointPosPub, {data=jointPos})
    -- -- simROS2.publish(jointVelPub, {data=jointVel})
    -- simROS2.publish(jointTorqPub, {data=jointTorque})
    -- simROS2.publish(imuPub, {data=imu})
    -- simROS2.publish(footContactPub, {data=footContact})
    simROS2.publish(tuneParamPub, {data=tuneParameters})
        
    -- -- draw legtrajectories
    -- local p=sim.getObjectPosition(leg0Path_graph,-1)
    -- sim.addDrawingObjectItem(leg0Trace,p)

    -- -- force vector transformation
    -- -- print("-----------")
    -- for i = 1, 6 do 
    --     tranMatrix[i] = sim.getObjectMatrix(footContactHandle[i], sphereBaseHandle)
    --     neg_force = {0,0,0}
    --     -- print(neg_force)
    --     for n = 1, 3 do 
    --         neg_force[n] = fc_force[i][n]*-1 
    --     end
    --     vector_w[i] = sim.multiplyVector(tranMatrix[i], neg_force)
    --     -- print(vector_w[i])
    -- end

    -- -- Ball Position
    -- ball_pos = sim.getObjectPosition(ballHandle, -1)
    -- ball_mass = sim.getShapeMass(ballHandle)
    -- robot_pos = sim.getObjectPosition(robotHandle, -1)
    -- -- print(ball_pos)
    -- -- print("-----------")
    -- -- Experiment data system ---
    -- -- j_angle_file:write(string.format('%.3f ',sim.getSimulationTime()+sim.getSimulationTimeStep())) -- timeStamp
    -- -- for i = 1, 19 do 
    -- --     j_angle_file:write(string.format('%.4f ',jointPos[i]))
    -- -- end    
    -- -- for i = 1, 19 do 
    -- --     j_angle_file:write(string.format('%.4f ',jointPosTarget[i]))
    -- -- end
    -- -- for i = 1, 19 do 
    -- --     j_angle_file:write(string.format('%.2f ',jointTorque[i]))
    -- -- end
    -- -- for i = 1, 6 do 
    -- --     for j = 1, 3 do 
    -- --         j_angle_file:write(string.format('%.2f ',fc_force[i][j]))
    -- --     end
    -- -- end
    -- -- for i = 1, 6 do 
    -- --     for j = 1, 3 do 
    -- --         j_angle_file:write(string.format('%.2f ',vector_w[i][j]))
    -- --     end
    -- -- end
    -- -- for i = 1, 9 do 
    -- --     j_angle_file:write(string.format('%.2f ',imu[i]))
    -- -- end
    -- -- for i = 1, 3 do 
    -- --     j_angle_file:write(string.format('%.3f ',ball_pos[i]))
    -- -- end
    -- -- for i = 1, 3 do 
    -- --     j_angle_file:write(string.format('%.3f ',robot_pos[i]))
    -- -- end
    
    -- -- j_angle_file:write(string.format('%.3f ',ball_mass))


    -- -- j_angle_file:write(string.format(' \n' ))
    -- ------------------------------

    -- -- plot data on graph
    -- sim.setGraphUserData(jointPos_graph, "jointTarget", jointPosTarget[7]+1.5)
    -- -- sim.setGraphUserData(jointPos_graph, "jointFeedback", jointPos[7])
    -- sim.setGraphUserData(orientation_graph, "roll", roll_filter)
    -- sim.setGraphUserData(orientation_graph, "pitch", pitch)
    -- sim.setGraphStreamValue(ballpos_graph,ballposx, ball_pos[1])
    -- sim.setGraphStreamValue(ballpos_graph,ballposy, ball_pos[2])
    -- -- sim.setGraphStreamValue(trackingErrBC_graph,jointErrBC1, jointPosTarget[1]-jointPos[1])
    -- -- sim.setGraphStreamValue(trackingErrBC_graph,jointErrBC2, jointPosTarget[2]-jointPos[3])
    -- -- sim.setGraphStreamValue(trackingErrBC_graph,jointErrBC3, jointPosTarget[3]-jointPos[3])
    -- -- sim.setGraphStreamValue(trackingErrCF_graph,jointErrCF1, jointPosTarget[7]-jointPos[7])
    -- -- sim.setGraphStreamValue(trackingErrCF_graph,jointErrCF2, jointPosTarget[8]-jointPos[8])
    -- -- sim.setGraphStreamValue(trackingErrCF_graph,jointErrCF3, jointPosTarget[9]-jointPos[9])
    -- -- sim.setGraphStreamValue(jointTorqueBC_graph,jointTorqBC1, jointTorque[1])
    -- -- sim.setGraphStreamValue(jointTorqueBC_graph,jointTorqBC2, jointTorque[2])
    -- -- sim.setGraphStreamValue(jointTorqueBC_graph,jointTorqBC3, jointTorque[3])
    -- -- sim.setGraphStreamValue(jointTorqueCF_graph,jointTorqCF1, jointTorque[7])
    -- -- sim.setGraphStreamValue(jointTorqueCF_graph,jointTorqCF2, jointTorque[8])
    -- -- sim.setGraphStreamValue(jointTorqueCF_graph,jointTorqCF3, jointTorque[9])

    -- -- Step simulation counter
    -- print("Simulation Counter : ", sim_counter)
    -- sim_counter = sim_counter + 1    
    
    -- Change mass Pitch Control Experiment
    -- if sim_counter == 50 then   --transf = sim.getObjectMatrix(obj,sim.handle_parent)
    --     M = sim.getShapeMass(ballHandle)
    --     inertiaMatrix, transformationMatrix=sim.getShapeInertia(ballHandle)
    --     print("====Start====")
    -- end

    -- if sim_counter == 700 then
    --     print('====Change1====')   --transf = sim.getObjectMatrix(obj,sim.handle_parent)
    --     -- newinertiaMatrix = {}
    --     -- newtransformationMatrix = {}
    --     -- for i, j in ipairs ( inertiaMatrix ) do
    --     --     newinertiaMatrix[i] = 64 * j
    --     -- end        
    --     -- for i, j in ipairs ( transformationMatrix ) do
    --     --     newtransformationMatrix[i] = 64 * j
    --     -- end
    --     sim.setShapeMass(ballHandle, M*8)
    --     -- sim.setShapeInertia(ballHandle, inertiaMatrix, transformationMatrix)
    -- end

    -- if sim_counter == 1100 then
    --     print('====Change2====')   --transf = sim.getObjectMatrix(obj,sim.handle_parent)
    --     sim.setShapeMass(ballHandle, M*64)
    --     -- sim.setShapeInertia(ballHandle, inertiaMatrix, transformationMatrix)
    -- end
    
    -- Stop Simulation by counter ---

    -- Trigger Roll Control ---
    -- if(activate_rollControl and sim_counter > 600) then
    --     simUI.setCheckboxValue(ui, 80, 2)
    --     checkboxChange(ui, 80, 2)
    --     activate_rollControl = false
    -- end
    -- if(sim_counter > 1500) then
    --     print("Stop Simulation from counter")
    --     simROS2.publish(terminateControllerPub,{data=true})       
    --     sim.stopSimulation()
    -- end
    ---------------------------------
end

function sysCall_cleanup()
    -- do some clean-up here
    -- sim.setShapeMass(ballHandle, M)
    -- sim.setShapeInertia(ballHandle, inertiaMatrix, transformationMatrix)
    
    -- simUI.destroy(ui)
    if simROS2 then
        simROS2.publish(terminateControllerPub,{data=true})

        simROS2.shutdownSubscription(motorComSub)
        simROS2.shutdownSubscription(motorMaxTorqueComSub)
        simROS2.shutdownPublisher(jointPosPub)
        -- simROS2.shutdownPublisher(jointVelPub)
        simROS2.shutdownPublisher(robotPosPub)
        simROS2.shutdownPublisher(jointTorqPub)
        simROS2.shutdownPublisher(tuneParamPub)
        simROS2.shutdownPublisher(imuPub) 
        simROS2.shutdownPublisher(footContactPub) 
    end
    -- simROS2.shutdownPublisher(triggerEpPub)

    -- File system
    -- j_angle_file:close()

end

-- See the user manual or the available code snippets for additional callback functions and details
