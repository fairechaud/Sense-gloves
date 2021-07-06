// Copyright 2021 Juan Carlos Orozco Arena jcorozco@acelab.com
// Uses SharpOSC library: https://github.com/ValdemarOrn/SharpOSC

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using SharpOSC;
using HTC.UnityPlugin.Vive;
using HTC.UnityPlugin.Utility;

public class RightControllerScript : MonoBehaviour
{
    private UDPSender sender;
    // UdpClient client = new UdpClient();

    public float interpolationPeriod = 0.05f;

    Vector3 wrist1UL;
    Vector3 wristOrientation1L;
    Transform spWristUL;
    Transform spShoulderL;
    Transform spElbowL;
    Transform spWristL;
    Hand hand;
    float fingersAngle = 0.0F;
    string cmd;
    bool initialTrackersRecorded = false;
    RigidPose chest;
    RigidPose rawInitialTrackerChest;
    RigidPose rawTrackerChest;
    RigidPose initialTrackerGlove;
    RigidPose trackerGlove;
    RigidPose glove;
    Vector3 fHandPos;
    Vector3 handPos;
    Quaternion handRot;
    float ScaleAngle1(float angle){
        float ret = angle;
        if(ret > 180) {
            ret = ret - 360;
        }
        ret = ret*1.0f;
        return ret;
    }

    void MoveRightArm() {
        //print("DEBUG: MoveFace()");
        //print(transform.forward);
        Quaternion controllerOrientation = transform.rotation * Quaternion.Euler(90, 0, 0);
        Vector3 controllerPosition = transform.position;
        // Quaternion controllerOrientation = handRot;
        // Vector3 controllerPosition = fHandPos;
        // controllerPosition.y = Lib1.getFloorHeadsetHeight() + transform.position.y;
        // print("Right: "+controllerPosition);
        // print(controllerPosition);

        wrist1UL = controllerPosition;
        
        // JCOA: Note that this makes the robot arm and the controller face the same direction.
        //Vector3 o1 = controllerOrientation.eulerAngles;
        //controllerOrientation.eulerAngles = new Vector3(o1.x, -o1.y, -o1.z);
        controllerOrientation *= Quaternion.Euler(90, 0, 90);
        wristOrientation1L = controllerOrientation.eulerAngles;

        var elbowWrist = Lib1.getElbowWrist(Lib1.mirrorX(wrist1UL));
        var T1_7 = Lib1.getAxisAngles(elbowWrist, hand, wristOrientation1L, false);

        simulateArm(T1_7);
    }
    
    void Start() {
        // Units in m and radians

        // Unity:
        // x axis between shulders. Positive to the right
        // y axis up-down. Positive up
        // z axis front-back. Positive front

        // This complies with right hand rule. On 3 axis and rotation.
        // Kinematics:
        // x axis front-back. Positive front (sticking out from chest)
        // y axis between shulders. Positive to the left shoulder
        // z axis up-down. Positive up

        ///arm_length_factor = 0.95F*robot_arm_length/arm_length;

        // Notes:
        // SR: Shoulder reference: Uses frame0 orientation (Y Axis parallel to the floor)
        // F0: Frame 0 reference. Chest axis.
        // F1: Frame 1 reference. Center on chest, rotated 25degrees on X0
        // Floor: _F0 displaced on z 1.000-0.0776476532296187
        // 
        // Use _SR to calculate elbow position (elbow out using horizontal plane from shoulder to wrist)
        //    

        // Shoulder to floor sitted down: 1.000
        // Chest axis reference to shoulder: # [x,y,z] -> [0,166.515929712244,77.6476532296187]
        // Floor reference to chest (not rotated) => [0,0,1000-77.6476532296187]
        ///sitting_chest_height = sitting_shoulder_height-0.0776476532296187F;

        ////var wrist1 = new Vector3(0.7000,0.0,0.0)
        wrist1UL = new Vector3(0.166515929712244F, 1.0F, 0.7000F);
        ////var wrist1 = new Vector3(2800.0,0.0,0.0)
        //var wrist1L = new Vector3(2.800F, 0.166515929712244F, 1.0F);
        ////var wrist1 = new Vector3(300.0,100.0,0.0)
        //var wrist1L = new Vector3(0.300, 0.100F+0.166515929712244F, 1.0F)
        ////var wrist1 = new Vector3(300.0,0.0,-100.0)
        //var wrist1L = new Vector3(0.300F, 0.166515929712244F, 1.0F-0.100F)

        var elbowWrist = Lib1.getElbowWrist(Lib1.mirrorX(wrist1UL));
        print(elbowWrist);

        wristOrientation1L = new Vector3(90,0,90); // x,y,z euler angles

        var MU_0L = Lib1.getMU_0L();
        var spSize = 0.05F;
        var spShoulderL = Lib1.NewSphere(Vector3.one*spSize, Lib1.mirrorX(Lib1.SR2U(Vector3.zero)), Color.red);
        spElbowL = Lib1.NewSphere(Vector3.one*spSize, Lib1.mirrorX(Lib1.SR2U(elbowWrist.Elbow)), Color.green);
        //spWristL = Lib1.NewSphere(Vector3.one*spSize, SR2U(elbowWrist.Wrist), Color.blue);
        spWristUL = Lib1.NewSphere(Vector3.one*spSize, Lib1.mirrorX(wrist1UL), new Color(1.0F, 1.0F, 0.0F, 0.5F));

        hand = new Hand(Lib1.SR2U(elbowWrist.Wrist), new Quaternion(), false);

        // TODO JCOA: Update address and port to project server (Jetson Nano on robot).
        // Note JCOA: We can create a sender (client to the same port) in each part.
        //            So we can copy this code to the left arm
        sender = new SharpOSC.UDPSender("10.0.0.20", 9015);
        // sender = new SharpOSC.UDPSender("192.168.100.63", 9001);
        //client.Connect(new IPEndPoint(IPAddress.Parse("192.168.1.128"),20777));

        InvokeRepeating("MoveRightArm", 1.0f, interpolationPeriod);
    }

    void Update() {
        rawTrackerChest = VivePose.GetPoseEx(TrackerRole.Tracker1);
        trackerGlove = VivePose.GetPoseEx(TrackerRole.Tracker2);
        if(Input.GetKeyDown("h")){
            if(VivePose.IsValidEx(TrackerRole.Tracker1) && VivePose.IsValidEx(DeviceRole.Hmd))// && VivePose.IsValidEx(TrackerRole.Tracker2))// && VivePose.IsValidEx(TrackerRole.Tracker3))
            {
                Debug.Log("Right hand calibrated.");
                rawInitialTrackerChest = rawTrackerChest;
                initialTrackerGlove = trackerGlove;
                initialTrackerGlove.rot = Quaternion.Inverse(initialTrackerGlove.rot) * rawInitialTrackerChest.rot;
                initialTrackersRecorded = true;
            }
        }
        if(initialTrackersRecorded && VivePose.IsValidEx(TrackerRole.Tracker1) && VivePose.IsValidEx(DeviceRole.Hmd))// && VivePose.IsValidEx(TrackerRole.Tracker2))// && VivePose.IsValidEx(TrackerRole.Tracker3))
        {
            chest.rot = Quaternion.Inverse(rawTrackerChest.rot) * rawInitialTrackerChest.rot;
            glove.rot = trackerGlove.rot * initialTrackerGlove.rot;
            glove.rot = Quaternion.Inverse(glove.rot) * rawTrackerChest.rot;
            fHandPos = Quaternion.Inverse(chest.rot)*(transform.position-rawTrackerChest.pos);
            fHandPos.y += 0.8F;
            handRot = glove.rot;
        }
        // if (OVRInput.Get(OVRInput.Button.Three)) {
        //     print("Button.Three");
        // }

        // For debuging purposes
        if (Input.GetKeyDown("1")) {
            print(transform.localEulerAngles);
            print(transform.position);
            MoveRightArm();
        }

        var elbowWrist = Lib1.getElbowWrist(Lib1.mirrorX(wrist1UL));
        spWristUL.localPosition = wrist1UL;
        spElbowL.localPosition = Lib1.mirrorX(Lib1.SR2U(elbowWrist.Elbow));
    }

    void simulateArm(float[] angles) {
        var j1c = -angles[0]*180.0F/Mathf.PI;
        var j2c = angles[1]*180.0F/Mathf.PI+(90.0F+25.0F);
        var j3c = -angles[2]*180.0F/Mathf.PI;
        var j4c = -angles[3]*180.0F/Mathf.PI;
        var j5c = -((angles[4]*180.0F/Mathf.PI) % 360.0F);
        var j6c = ((angles[5]*180.0F/Mathf.PI) % 360.0F);
        var j7c = -((angles[6]*180.0F/Mathf.PI) % 360.0F);

        // cmd = "<1.X.1."+(int)(j1c)+"."+(int)(-j2c)+"."+(int)j3c+"."+(int)(-j4c)+"."+(int)j5c+"."+(int)(-j6c)+">";
        // if (cmd!=null)
        // {
        //     byte[] bytesent = Encoding.ASCII.GetBytes(cmd);
        //     // client.Send(bytesent,bytesent.Length);
        //     print("Right arm: "+cmd);
        // }
        // cmd = "<1.X.1."+(int)(j1c)+"."+(int)(-j2c)+"."+(int)j3c+"."+(int)(-j4c)+"."+(int)j5c+"."+(int)(-j6c)+">";
        var command = 1; // Set angles
        var message = new SharpOSC.OscMessage("/ArmR", command, j1c, -j2c, j3c, -j4c, j5c, -j6c, j7c);
        if(sender != null)
        {
            sender.Send(message);
        }
    }
}
