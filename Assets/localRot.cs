using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using HTC.UnityPlugin.Vive;
using HTC.UnityPlugin.Utility;


using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
//using System.IO.Ports;
using System.Threading;

public class localRot : MonoBehaviour
{
    bool initialTrackersRecorded = false;
    RigidPose initialTrackerChest;
    RigidPose trackerChest;
    RigidPose chest;
    RigidPose rawChest;
    RigidPose rawInitialTrackerChest;
    RigidPose rawTrackerChest;
    RigidPose initialTrackerShoulderR;
    RigidPose trackerShoulderR;
    RigidPose  shoulderR;
    Vector3 eulerShoulderR;
    Vector3 fEulerShoulderR;
    RigidPose initialTrackerShoulderL;
     RigidPose trackerShoulderL;
    RigidPose  shoulderL;
    Vector3 eulerShoulderL;
    Vector3 fEulerShoulderL;
    RigidPose initialTrackerElbowR;
    RigidPose trackerElbowR;
    RigidPose  elbowR;
    Vector3 eulerElbowR;
    Vector3 fEulerElbowR;

    // Start is called before the first frame update
    
    //Vector3 rH;
    //define a client
    UdpClient client = new UdpClient();
    void Start()
    {
        //find a server in address XXX.XXX.X.X port XXXX
        client.Connect(new IPEndPoint(IPAddress.Parse("192.168.0.177"),8080));

        // mf = GetComponent<MeshFilter>();
        // orChest =mf.mesh.vertices;
        // leftHand = new Vector3[orChest.Length];
    }

    // Update is called once per frame
    void Update()
    {
        //create a string message for left hand positions
        // string leftMessage = ("x: "+leftPos[0]+"y: "+leftPos[1]+"z: "+leftPos[2]);
        // if(Input.GetKeyDown("z"))
        // {     
        //     Debug.Log("Left position: "+leftPos);
        //     if (leftMessage!=null)
        //             {
        //                 //send the left hand message
        //                 byte[] bytesent = Encoding.ASCII.GetBytes(leftMessage);
        //                 client.Send(bytesent,bytesent.Length);
        //                 Debug.Log("Message"+leftMessage+" sent with success");
        //             }
        // }
        rawTrackerChest = VivePose.GetPoseEx(TrackerRole.Tracker1);
        trackerShoulderR = VivePose.GetPoseEx(TrackerRole.Tracker2);
        //trackerShoulderL = VivePose.GetPoseEx(TrackerRole.Tracker8);
        trackerElbowR = VivePose.GetPoseEx(TrackerRole.Tracker3);

        trackerChest = rawTrackerChest;
        //rightTrans = VivePose.GetPoseEx(TrackerRole.Tracker2);
        
          
             /* var rH = localPosRight.getRightPosition();
             string rightMessage = ("x: "+rH[0]+", y: "+rH[1]+", z: "+rH[2]);
             //Debug.Log("Right position: "+handPosR);
             if (rightMessage!=null)
                     {
                         //send the left hand message
                        byte[] bytesent = Encoding.ASCII.GetBytes(rightMessage);
                         client.Send(bytesent,bytesent.Length);
                        Debug.Log("Message"+rightMessage);
                    } */
         
        
        if(Input.GetKeyDown("l")){
            if(VivePose.IsValidEx(TrackerRole.Tracker1) && VivePose.IsValidEx(DeviceRole.Hmd) && VivePose.IsValidEx(TrackerRole.Tracker2))// && VivePose.IsValidEx(TrackerRole.Tracker3))
            {
                Debug.Log("Calibrated with current pose.");
                //trackerChest = VivePose.GetPoseEx(TrackerRole.Tracker1);
                // trackerHead = VivePose.GetPoseEx(DeviceRole.Hmd);
                // trackerRight = VivePose.GetPoseEx(TrackerRole.Tracker2);
                // trackerLeft = VivePose.GetPoseEx(TrackerRole.Tracker3);
                //trackerRElbow = VivePose.GetPoseEx(TrackerRole.Tracker2);
                initialTrackerChest = trackerChest;
                rawInitialTrackerChest = rawTrackerChest;
                // initialTrackerHead = trackerHead;
                // initialTrackerHead.rot = Quaternion.Inverse(initialTrackerHead.rot) * initialTrackerChest.rot;
                // initialTrackerRight = trackerRight;
                // initialTrackerRight.rot = Quaternion.Inverse(initialTrackerRight.rot) * initialTrackerChest.rot;
                // initialTrackerLeft = trackerLeft;
                // initialTrackerLeft.rot = Quaternion.Inverse(initialTrackerLeft.rot) * initialTrackerChest.rot;
                initialTrackerShoulderR = trackerShoulderR;
                initialTrackerElbowR = trackerElbowR;
                initialTrackerElbowR.rot = Quaternion.Inverse(initialTrackerElbowR.rot) * initialTrackerShoulderR.rot;
                initialTrackerShoulderR.rot = Quaternion.Inverse(initialTrackerShoulderR.rot) * rawInitialTrackerChest.rot;
                initialTrackerShoulderL = trackerShoulderL;
                initialTrackerShoulderL.rot = Quaternion.Inverse(initialTrackerShoulderL.rot) * rawInitialTrackerChest.rot;
                initialTrackersRecorded = true;
            }
        }
        
        if(Input.GetKeyDown("2")){
            //Debug.Log("TestR");
            if(initialTrackersRecorded && VivePose.IsValidEx(TrackerRole.Tracker1) && VivePose.IsValidEx(DeviceRole.Hmd) && VivePose.IsValidEx(TrackerRole.Tracker2))// && VivePose.IsValidEx(TrackerRole.Tracker3))
            {
                rawChest.rot = Quaternion.Inverse(rawTrackerChest.rot) * rawInitialTrackerChest.rot;// * rotAxisChest;
                chest.rot = Quaternion.Inverse(trackerChest.rot) * initialTrackerChest.rot;// * rotAxisChest;
                // head.rot = trackerHead.rot * initialTrackerHead.rot;
                // head.rot = Quaternion.Inverse(head.rot) * rawTrackerChest.rot;
                // right.rot = trackerRight.rot * initialTrackerRight.rot;
                // right.rot = Quaternion.Inverse(right.rot) * trackerChest.rot;
                shoulderR.rot = trackerShoulderR.rot * initialTrackerShoulderR.rot;
                shoulderR.rot = Quaternion.Inverse(shoulderR.rot) * rawTrackerChest.rot;
                shoulderL.rot = trackerShoulderL.rot * initialTrackerShoulderL.rot;
                shoulderL.rot = Quaternion.Inverse(shoulderL.rot) * rawTrackerChest.rot;
                elbowR.rot = trackerElbowR.rot * initialTrackerElbowR.rot;
                elbowR.rot = Quaternion.Inverse(elbowR.rot) * trackerShoulderR.rot;
//                trackerChest = VivePose.GetPoseEx(TrackerRole.Tracker1);
                // trackerHead = VivePose.GetPoseEx(DeviceRole.Hmd);
                // trackerRight = VivePose.GetPoseEx(TrackerRole.Tracker2);
                // trackerLeft = VivePose.GetPoseEx(TrackerRole.Tracker3);
//                trackerRelbow = VivePose.GetPoseEx(TrackerRole.Tracker2);

//                chest.rot = Quaternion.Inverse(trackerChest.rot) * initialTrackerChest.rot;
                // head.rot = trackerHead.rot * initialTrackerHead.rot;
                // head.rot = Quaternion.Inverse(head.rot) * trackerChest.rot;
                // right.rot = trackerRight.rot * initialTrackerRight.rot;
                // right.rot = Quaternion.Inverse(right.rot) * trackerChest.rot;
                // left.rot = trackerLeft.rot * initialTrackerLeft.rot;
                // left.rot = Quaternion.Inverse(left.rot) * trackerChest.rot;
//                rElbow.rot = trackerRElbow.rot * initialTrackerRElbow.rot;
//                rElbow.rot = Quaternion.Inverse(rElbow.rot) * trackerChest.rot;
                
 //               chest.pos = trackerChest.rot*shiftChest + trackerChest.pos - initialTrackerChest.pos;
                // head.pos = trackerHead.rot*shiftHead + trackerHead.pos - trackerChest.pos;
                // right.pos = trackerRight.rot*shiftRight + trackerRight.pos - trackerChest.pos;
                // left.pos = trackerLeft.rot*shiftLeft + trackerLeft.pos - trackerChest.pos;

 //               fEulerRElbow = chest.rot.eulerAngles;
 //               eulerRElbow.x = Convert.ToInt16(fEulerRElbow.x);
 //               eulerRElbow.y = Convert.ToInt16(fEulerRElbow.y);
 //               eulerRElbow.z = Convert.ToInt16(fEulerRElbow.z);
                

 //               Debug.Log(""
                // +"Chest pos: "+chest.pos.x+" "+chest.pos.y+" "+chest.pos.z
                // +"\nHead pos: "+head.pos.x+" "+head.pos.y+" "+head.pos.z
                // +"\nRight pos: "+right.pos.x+" "+right.pos.y+" "+right.pos.z
                // +"\nLeft pos: "+left.pos.x+" "+left.pos.y+" "+left.pos.z
                // +"\nChest rot: "+chest.rot.w+" "+chest.rot.x+" "+chest.rot.y+" "+chest.rot.z
                // +"\nHead rot: "+head.rot.w+" "+head.rot.x+" "+head.rot.y+" "+head.rot.z
                // +"\nRight rot: "+right.rot.w+" "+right.rot.x+" "+right.rot.y+" "+right.rot.z
                // +"\nLeft rot: "+left.rot.w+" "+left.rot.x+" "+left.rot.y+" "+left.rot.z
//                +"Right Elbow Euler: "+eulerRElbow.x+" "+eulerRElbow.y+" "+eulerRElbow.z+"\n"
//                );
                fEulerShoulderR = shoulderR.rot.eulerAngles;
                eulerShoulderR.x = Convert.ToInt16(fEulerShoulderR.x);
                eulerShoulderR.y = Convert.ToInt16(fEulerShoulderR.y);
                eulerShoulderR.z = Convert.ToInt16(fEulerShoulderR.z);

                fEulerShoulderL = shoulderL.rot.eulerAngles;
                eulerShoulderL.x = Convert.ToInt16(fEulerShoulderL.x);
                eulerShoulderL.y = Convert.ToInt16(fEulerShoulderL.y);
                eulerShoulderL.z = Convert.ToInt16(fEulerShoulderL.z);

                fEulerElbowR = elbowR.rot.eulerAngles;
                eulerElbowR.x = Convert.ToInt16(fEulerElbowR.x);
                eulerElbowR.y = Convert.ToInt16(fEulerElbowR.y);
                eulerElbowR.z = Convert.ToInt16(fEulerElbowR.z);
         
                if(eulerShoulderR.x>180){
                    eulerShoulderR.x -= 360;
                }
                if(eulerShoulderR.y>180){
                    eulerShoulderR.y -= 360;
                }
                if(eulerShoulderR.z>180){
                    eulerShoulderR.z -= 360;
                }
                // Debug.Log(eulerShoulderR.x+","+eulerShoulderR.y+","+eulerShoulderR.z);
             
                if(eulerShoulderL.x>180){
                    eulerShoulderL.x -= 360;
                }
                if(eulerShoulderL.y>180){
                    eulerShoulderL.y -= 360;
                }
                if(eulerShoulderL.z>180){
                    eulerShoulderL.z -= 360;
                }

                if(eulerElbowR.x>180){
                    eulerElbowR.x -= 360;
                }
                if(eulerElbowR.y>180){
                    eulerElbowR.y -= 360;
                }
                if(eulerElbowR.z>180){
                    eulerElbowR.z -= 360;
                }
               // Debug.Log(eulerElbowR.x+","+eulerElbowR.y+","+eulerElbowR.z);
                // Debug.Log(eulerShoulderL.x+","+eulerShoulderL.y+","+eulerShoulderL.z);
                //CONVERSION DE LIMITES
                /*Segun yo a las 3 de la mañana el eje Z del tracker de la hombrera acciona theta2
                (brazo extendido de 0 hasta 115 grados) y el eje Y controla theta 1 (brazo hacia adelante o atras)*/ 

                //Acordar limite y sentidos de giro con Alex PENDIENTEEEEEE!!!!!!  

                /*Theta 5 y theta 6 se comparan desde la muñeca*/
                /*Theta 4 puede ser el resultado de comparar la posicion de la muñeca (SenseGlove) con un tracker fijo en el codo apuntando hacia atrás----> Theta 3 puede estar en funcion de la hombrera y el codo SINO, optar por el antebrazo*/

                /*Se sigue la convencion de chest referencia de hombro, hombro referencia de codo o antebrazo, y codo/antebrazo referencia de muñeca (SenseGlove)*/
                
                /*Podria compararse con chest (origen) si sirve de algo (?????)*/
                /*el siguiente codigo es copiado del ejemplo de head y chest, acordar nuevos limites o sentidos de direccion*/
                // if(eulerShoulderR.y < -0){
                //         eulerShoulderR.y = -0;
                //     }
                //     if(eulerShoulderR.y > 90){
                //         eulerShoulderR.y = 90;
                // }
                //      if(eulerShoulderL.y < -0){
                //         eulerShoulderL.y = -0;
                //     }
                //     if(eulerShoulderL.y > 90){
                //         eulerShoulderL.y = 90;
                // }
                /*Se definieron los siguientes indices:
                    chest           ----->      Tracker 1
                    handRight       ----->      Tracker 2
                    elbowRight      ----->      Tracker 3
                    shoulderRight   ----->      Tracker 4
                    handLeft        ----->      Tracker 5
                    elbowLeft       ----->      Tracker 6
                    shoulderLeft    ----->      Tracker 7
                Asi se pueden encender en orden seguido todos los trackers de un brazo y probarlo de forma aislada
                Teoricamente, el sentido en el que se ajusten no importa pero sera critico guardar una nocion de que ejes controlan ciertos movimientos de theta 1-6
                    POR EJEMPLO: El eje X del tracker montado en la hombrera no sirve para controlar motores porque su sentido es reemplazado por el giro del torso (ejes tomados del Developer Guidelines)
                /*Como se esta comparando directamente desde la libreria de HTC tracker, los datos de tracker 1..7 son globales y pueden ir en scripts separados (uno por elemento para tratar de no confundirnos y tener todo junto)*/
                /*Cada script puede llevar su definicion de servidor y mandar sus datos en paralelo, solo hace falta definir el nuevo servidor en cada script y mandar el string de comandos*/
                /*En assets/SenseGlove/Scripts/Tracking/SG_StopFingers.cs esta la funcion que envia los datos de las manos, ya generé las cadenas con los comandos en su lugar 
                SOLO HACE FALTA PROBAR EL PULGAR ####SE PUEDEN AJUSTAR LOS INDICES QUE PUSE PARA LOGRAR MEJOR SEGUIMIENTO DE POSICION###*/  
           
                /*Maximo 2 trackers podran ir conectados por bluetooth tomando en cuenta que los SenseGloves estaran conectados por wireless*/           
                /*Dejé mucho respaldos de este mismo proyecto en la carpeta de Documentos con varias etapas*/
                /*La camara ZED ya funciona, ya que se termine este proyecto, que el equipo de Nora agregue las configuraciones de cámara*/ 
                /*No sé en qué momento convenga enterar al equipo de esto pero es viable por que pronto se puede evaluar su efectividad en el robot*/
            }
        }

    }
}
