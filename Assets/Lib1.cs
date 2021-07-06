// Copyright 2021 Juan Carlos Orozco Arena

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Lib1
{
    public static float arm_length = 0.450F; // 0.6F
    public static float robot_arm_length = 0.52026F;
    public static float sitting_shoulder_height = 1.0F;
    public static float floor_headset_height = 1.16F;

    public static float getSittingChestHeight() {
        return sitting_shoulder_height-0.0776476532296187F;
    }
    public static float getFloorHeadsetHeight() {
        return floor_headset_height;
    }

    public static Matrix4x4 getMU_0L() {
        var MUL = new Matrix4x4(); // Unity axis to M0L
        MUL.SetRow(0, new Vector4(0,1,0,0));
        MUL.SetRow(1, new Vector4(0,0,1,getSittingChestHeight()));
        MUL.SetRow(2, new Vector4(1,0,0,0));
        MUL.SetRow(3, new Vector4(0,0,0,1));
        var M0L = getDH(0.0F,-65.0F*Mathf.PI/180.0F,0.0F,0.0F);
        return MUL*M0L;
    }

    public static Matrix4x4 getDH(float theta, float alfa, float d, float r) {
        var M1 = new Matrix4x4();
        M1.SetRow(0, new Vector4(Mathf.Cos(theta), -Mathf.Sin(theta)*Mathf.Cos(alfa), Mathf.Sin(theta)*Mathf.Sin(alfa), r*Mathf.Cos(theta)));
        M1.SetRow(1, new Vector4(Mathf.Sin(theta), Mathf.Cos(theta)*Mathf.Cos(alfa), -Mathf.Cos(theta)*Mathf.Sin(alfa), r*Mathf.Sin(theta)));
        M1.SetRow(2, new Vector4(0, Mathf.Sin(alfa), Mathf.Cos(alfa), d));
        M1.SetRow(3, new Vector4(0.0F, 0.0F, 0.0F, 1.0F));
        return M1;
    }
    public static Vector4 plane_point_normal(Vector3 point, Vector3 normal) {
        //point = np.array([0,0,0])
        //normal_z = np.array([0,0,1])
        // a plane is a*x+b*y+c*z+d=0
        // [a,b,c] is the normal. Thus, we have to calculate
        // d and we're set
        // d = -point.dot(normal)
        var d = -Vector3.Dot(point,normal);
        //return np.append(normal,d)
        return new Vector4(normal.x,normal.y,normal.z,d);
    }

    public static Vector4 plane_3points(Vector3 A, Vector3 B, Vector3 C) {
        Vector3 normal = Vector3.Cross(B-A, C-A);
        return plane_point_normal(A, normal);
    }

    public static Vector3[] get_plane_plane_intersection(Vector4 A, Vector4 B) {
        //U = normalized(numpy.cross(A[:-1], B[:-1]))
        Vector3 p1 = new Vector3(A.x, A.y, A.z);
        Vector3 p2 = new Vector3(B.x, B.y, B.z);
        Vector3 U = Vector3.Cross(p1,p2).normalized; 
        //Debug.Log("U: " + U);
        //M = numpy.array((A[:-1], B[:-1], U))
        Matrix4x4 M = new Matrix4x4();
        M.SetRow(0, new Vector4(A.x, A.y, A.z, 0));
        M.SetRow(1, new Vector4(B.x, B.y, B.z, 0));
        M.SetRow(2, new Vector4(U.x, U.y, U.z, 0));
        M.SetRow(3, new Vector4(0, 0, 0, 1));
        //Debug.Log("M: " + M);
        //X = numpy.array((-A[-1], -B[-1], 0.))
        Vector3 X = new Vector3(-A.w, -B.w, 0);
        //Xt = X.reshape(-1, 1)
        //Mi = numpy.linalg.inv(M)
        Matrix4x4 Mi = M.inverse;
        //Debug.Log("Mi: " + Mi);
        //print(U)
        //print(numpy.matmul(Mi,Xt))
        Vector3 p3 = Mi.MultiplyPoint(X);
        //return U, numpy.linalg.solve(M, X)
        return new Vector3[] {U, p3};
    }

    // Floor reference to Shoulder reference (_SR)
    public static Vector3 Floor2SR(Vector3 point1) {
        var point2 = new Vector3(0.0F,0.166515929712244F,sitting_shoulder_height);
        return point1-point2;
    }

    public static Vector3 SR2Floor(Vector3 point1) {
        var point2 = new Vector3(0.0F,0.166515929712244F,sitting_shoulder_height);
        return point1+point2;
    }

    // Shoulder reference to Frame1 reference
    // (y+183.73)=>z1, x=>x1, z=>-y1
    public static Vector3 SR2F1(Vector3 point1){
        return new Vector3(point1.x, -point1.z, point1.y+0.18373F);
    }

    // Shoulder reference to Unity reference
    // (y+183.73)=>z1, x=>x1, z=>-y1
    public static Vector3 SR2U(Vector3 point1){
        var point2 = new Vector3(0.0F,0.166515929712244F,sitting_shoulder_height);
        var point3 =F0toU(point1+point2);
        return point3;
    }

    // Frame0 to Unity reference
    public static Vector3 F0toU(Vector3 point1){
        return new Vector3(-point1.y, point1.z, point1.x);
    }

    // Unity to Frame0 reference
    public static Vector3 UtoF0(Vector3 point1){
        return new Vector3(point1.z, -point1.x, point1.y);
    }

    public static Vector3 mirrorX(Vector3 point1){
        return new Vector3(-point1.x, point1.y, point1.z);
    }

    public static Transform NewSphere(Vector3 scale, Vector3 position, Color color) {
        Transform shape = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
        UnityEngine.Object.Destroy(shape.GetComponent<Collider>()); // no collider, please!
        shape.localScale = scale; // set the rectangular volume size
        shape.position = position;
        //shape.rotation = Quaternion.LookRotation(dir, p2b - p1b);
        shape.GetComponent<Renderer>().material.color = color;
        shape.GetComponent<Renderer>().enabled = true; // show it
        return shape;
    }

    // Return [elbow1_angle, elbow1, wrist1]
    public static ElbowWrist getElbowWrist(Vector3 wrist1UL) {
        //var wrist1L = MUL.MultiplyPoint3x4(wrist1UL);
        var wrist1L = UtoF0(wrist1UL);

        //print("wrist1L "+wrist1L);

        var wrist1 = Floor2SR(wrist1L);
        //print("wrist1L_SR "+wrist1);


        // TODO: Make a function for this
        float wrist1_distance = wrist1.magnitude;
        if(wrist1_distance > arm_length){
            float scale = arm_length/wrist1_distance;
            wrist1 = wrist1*scale;
        }
        //print("arm_length_factor "+arm_length_factor);
        float arm_length_factor = 0.95F*robot_arm_length/arm_length;
        wrist1 = wrist1*arm_length_factor;

        Vector3 wrist1_mid = wrist1/2.0F;

        //print("wrist1 "+wrist1);
        //print("wrist1_mid "+wrist1_mid);

        // Left shoulder
        var shoulder = new Vector3(0.0F,0.0F,0.0F);

        // Shoulders out strategy (better reach working on tables)
        // Get 2 planes to calculate intersection line of shoulder position
        var horizontal_y = new Vector3(0.0F,1.0F,0.0F);
        Vector4 shoulder_plane1 = plane_3points(shoulder, wrist1, horizontal_y);
        //print("shoulder plane1 "+shoulder_plane1);
        Vector4 shoulder_plane2 = plane_point_normal(wrist1_mid, wrist1);
        //print("shoulder plane2 "+shoulder_plane2);
        var r1 = get_plane_plane_intersection(shoulder_plane1, shoulder_plane2);
        var n1 = r1[0];
        var v1 = r1[1];
        //print("n1, v1 "+r1[0]+", "+r1[1]);

        var co1 = wrist1.magnitude/2.0F;
        var h1 = robot_arm_length/2.0F;
        //print("co1, h1 "+co1+", "+h1);
        // Already checked this before.
        //if(co1 > h1):
        //    co1 = h1
        var half_elbow1_angle = Mathf.Asin(co1/h1);
        var elbow1_angle = half_elbow1_angle*2.0F;
        //print("elbow1_angle "+elbow1_angle*180.0/Mathf.PI);

        // h^2 = co^2+ca^2
        // ca = sqrt(h^2, ca^2)

        var ca1 = h1 * Mathf.Cos(half_elbow1_angle);
        //print("ca1 "+ca1);
        var elbow1 = n1*ca1+v1;
        var elbow1_length = elbow1.magnitude;
        //print("elbow1, elbow1_length "+elbow1+", "+elbow1_length);
        return new ElbowWrist(elbow1_angle, elbow1, wrist1);
    }
    public static float[] getAxisAngles(ElbowWrist elbowWrist, Hand hand, Vector3 wristOrientation1L, bool left) {
        var elbow1_angle = elbowWrist.ElbowAngle;
        var elbow1 = elbowWrist.Elbow;
        var wrist1 = elbowWrist.Wrist;
        float T1;

        Quaternion rotation = Quaternion.Euler(-25.0F, 0.0F, 0.0F);
        Matrix4x4 rm1 = Matrix4x4.Rotate(rotation);
        //Debug.Log("rm1 "+rm1);
        var elbow1_rot1 = rm1.MultiplyPoint(elbow1);
        //print("elbow1_rot1 "+elbow1_rot1);
        var wrist1_rot1 = rm1.MultiplyPoint(wrist1);

        //t2 = T2_z.subs(z,elbow1_rot1[1,0]+183.73)
        //t1 = T1_T2_y.subs(T2,t2).subs(y,-elbow1_rot1[2,0])
        var z1 = elbow1_rot1.y+0.18373F;
        //print("z1 "+z1);
        var y1 = -elbow1_rot1.z;
        //print("y1 "+y1);
        var x1 = -elbow1_rot1.x;
        //print("x1 "+x1);

        var T2 = Mathf.Acos(3.8442317302887F*z1 - 0.706300695805943F);
        var T1y = Mathf.Acos(-3.8442317302887F*y1/Mathf.Sin(T2));
        T1y = T1y-Mathf.PI; // Use if we use the y1 formula above        
        var T1x = Mathf.Asin(3.8442317302887F*x1/Mathf.Sin(T2));

        // TODO: Test rotate 180 degrees t1 (reverse axis). And change sign to t2
        T2 = -T2;

        var x_a = 0.26013F*Mathf.Asin(T1x)*Mathf.Asin(T2);
        var y_a = -0.26013F*Mathf.Asin(T2)*Mathf.Acos(T1x);
        var x_b = 0.26013F*Mathf.Asin(T1y)*Mathf.Asin(T2);
        var y_b = -0.26013F*Mathf.Asin(T2)*Mathf.Acos(T1y);

        var elbow1_F1 = SR2F1(elbow1_rot1);

        var pa = new Vector3(x_a, y_a, elbow1_F1.z);
        var pb = new Vector3(x_b, y_b, elbow1_F1.z);
        if((pa-elbow1_F1).magnitude < (pb-elbow1_F1).magnitude){
            T1 = T1x;
        } else {
            T1 = T1y;
        }

        var T4 = Mathf.PI-elbow1_angle;
        var z2 = wrist1_rot1.y+0.18373F;
        var T3 = -Mathf.Asin(0.000038442317302887F*(-100000.0F*z2 + 26013.0F*Mathf.Cos(T2)*Mathf.Cos(T4) + 26013.0F*Mathf.Cos(T2) + 18373.0F)/(Mathf.Sin(T2)*Mathf.Sin(T4)));
        //print("T1 "+T1*180.0F/Mathf.PI); // 90-j1
        //print("T2 "+T2*180.0F/Mathf.PI); // 90+j2
        //print("T3 "+T3*180.0F/Mathf.PI);
        //print("T4 "+T4*180.0F/Mathf.PI);

        //return new float[] {0, 0, 0, 0};

        //////////////////////////////
        var M1L = getDH(-Mathf.PI/2.0F+T1,-Mathf.PI/2.0F,0.18373F,0.0F);
        var M2L = getDH(T2,Mathf.PI/2.0F,0.0F,0.0F);
        var M3L = getDH(Mathf.PI/2.0F+T3,-Mathf.PI/2.0F,0.26013F,0.0F);
        var M4L = getDH(T4,Mathf.PI/2.0F,0.0F,0.0F);

        Quaternion wristQuaternion1L = new Quaternion();
        wristQuaternion1L.eulerAngles = wristOrientation1L;
        Matrix4x4 wristMatrixL = Matrix4x4.TRS(Vector3.zero,wristQuaternion1L,Vector3.one);

        // Use homogeneeus matrices instead of rotation matrices (Matrix4x4 is more efficient in Unity)
        // 3 Formulas for inverse kinematics [row,column] starts on [0,0]:
        // R_ = (R0_4L_**-1)*R0_7L (R0_7L desired rotation of end effector)
        var MU_0L = getMU_0L();
        var MU_4L = MU_0L*M1L*M2L*M3L*M4L;
        var MU_4Li = MU_4L.inverse;
        var R1L = MU_4Li*wristMatrixL;

        var T5 = Mathf.Atan2(-R1L[0,2],R1L[1,2]); // +k*pi
        var T7 = Mathf.Atan2(-R1L[2,1],R1L[2,0]);
        var T6 = Mathf.Asin(R1L[2,2]);

        //T5 = 0.0F;
        //T6 = 0.0F;
        //T7 = 0.0F;

        var M5L = getDH(-Mathf.PI/2.0F+T5,-Mathf.PI/2.0F,0.07853F,0.0F);
        var M6L = getDH(-Mathf.PI/2.0F+T6,Mathf.PI/2.0F,0.0F,0.0445F);
        var M7L = getDH(T7,0.0F,0.0F,0.071F);
        var M7LB = getDH(T7,0.0F,0.0F,0.13197F);

        var MU_5L = MU_0L*M1L*M2L*M3L*M4L*M5L;
        var M1_6L = M1L*M2L*M3L*M4L*M5L*M6L;
        var M1_7L = M1_6L*M7L;
        var M1_7LB = M1_6L*M7LB;
        //Debug.Log("M1_7LB: " + M1_7LB);
        var MU_7L = MU_0L*M1_7L;
        var MU_7LB = MU_0L*M1_7LB;
        //Debug.Log("MU_7LB: " + MU_7LB);

        Vector3 handPositionL;
        Quaternion handRotationL;
        if(left) {
            handPositionL = new Vector3(-MU_7L[0,3], MU_7L[1,3], MU_7L[2,3]);
            Quaternion qy = new Quaternion(1,0,0,0);
            handRotationL = qy*MU_7L.rotation*qy;
        } else {
            handPositionL = new Vector3(MU_7L[0,3], MU_7L[1,3], MU_7L[2,3]);
            handRotationL = MU_7L.rotation;
        }
        //print(MU_7L.rotation.eulerAngles);

        hand.hand.localPosition = handPositionL; // SR2U(elbowWrist.Wrist);
        hand.hand.rotation = handRotationL;  // wristOrientation1L;

        return new float[] {T1, T2, T3, T4, T5, T6, T7};
    }
}

public class ElbowWrist
{
    public float ElbowAngle;
    public Vector3 Elbow;
    public Vector3 Wrist;
    public ElbowWrist(float elbow_angle, Vector3 elbow, Vector3 wrist) {
        ElbowAngle = elbow_angle;
        Elbow = elbow;
        Wrist = wrist;
    }
}

public class Hand {
    public Transform hand;
    Transform thumbPivot;
    Transform fingersPivot;
    Quaternion rotation;
    Vector3 position;
    bool left;
    public Hand(Vector3 position_, Quaternion rotation_, bool left_) {
        position = position_;
        rotation = rotation_;
        left = left_;

        hand = (new GameObject("hand")).transform;
        var rot1 = (new GameObject("hand")).transform;
        
        var thumb = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
        UnityEngine.Object.Destroy(thumb.GetComponent<Collider>()); // no collider, please!
        var thumb_len = 0.065F;
        thumb.localScale = new Vector3(thumb_len,0.020F,0.020F); // set the rectangular volume size
        if(left) {
            thumb.position = new Vector3(-thumb_len/2,0.0F,0.0F);
        } else {
            thumb.position = new Vector3(thumb_len/2,0.0F,0.0F);
        }
        thumb.GetComponent<Renderer>().material.color = Color.white;

        thumbPivot = (new GameObject("thumbPivot")).transform;
        thumb.parent = thumbPivot;
        if(left) {
            thumbPivot.position = new Vector3(-0.030F,0.0F,-0.015F);
        } else {
            thumbPivot.position = new Vector3(0.030F,0.0F,-0.015F);
        }

        var fingers = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
        UnityEngine.Object.Destroy(fingers.GetComponent<Collider>()); // no collider, please!
        fingers.localScale = new Vector3(0.070F,0.020F,0.080F); // set the rectangular volume size
        fingers.position = new Vector3(0.0F,0.0F,0.040F);
        //fingers.rotation = Quaternion.LookRotation(dir, p2b - p1b);
        fingers.GetComponent<Renderer>().material.color = Color.white;

        fingersPivot = (new GameObject("thumbPivot")).transform;
        fingers.parent = fingersPivot;
        fingersPivot.position = new Vector3(0.0F,0.0F,0.035F);

        var palm = GameObject.CreatePrimitive(PrimitiveType.Cube).transform;
        UnityEngine.Object.Destroy(palm.GetComponent<Collider>()); // no collider, please!
        palm.localScale = new Vector3(0.070F,0.020F,0.080F); // set the rectangular volume size
        palm.GetComponent<Renderer>().material.color = Color.grey;
        palm.GetComponent<Renderer>().enabled = true; // show it

        thumbPivot.parent = rot1;
        palm.parent = rot1;
        fingersPivot.parent = rot1;
        rot1.parent = hand;

        // Hand angles at T0 to T7 = 0 (65.0, 90.0, 90.0)
        Quaternion r1 = new Quaternion();
        if(left) {
            r1.eulerAngles = new Vector3(180.0F,90.0F,90.0F);
        } else {
            r1.eulerAngles = new Vector3(0.0F,90.0F,90.0F);
        }
        rot1.rotation = r1;

        hand.position = position;
        hand.rotation = rotation;
    }

    public void setFingersAngle(float angle){
        // Close hand from 0 to 110
        Debug.Log(angle);
        if(left){
            thumbPivot.localEulerAngles = new Vector3(0.0F, 0.0F, -angle);
        } else {
            thumbPivot.localEulerAngles = new Vector3(0.0F, 0.0F, angle);
        }
        fingersPivot.localEulerAngles = new Vector3(-angle, 0.0F, 0.0F);
    }
}
