using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Text;
using SG;
using SGCore.SG;
using System.Net;
using System.Net.Sockets;
//using System.IO.Ports;
using System.Threading;

namespace SG
{
	
	/// <summary> Prevents fingers with SG_FingerFeedback colliders from moving through objects marked with SG_Material by locking the AnimationLayer. </summary>
	public class SG_StopFingers : MonoBehaviour
	{

		//--------------------------------------------------------------------------------------------------------------------------
		// Member Variables

		/// <summary> HandAnimator reference to stop the finger from flexing. </summary>
		public SG_HapticGlove senseGlove;
		public SG_HapticGlove leftGlove, rightGlove;
		public SG_HandAnimator handAnimation;

		UdpClient clientManos = new UdpClient();//create a client
		byte[] bytesent;
		public string cmdRight;

		public string cmdLeft;
		public float fingConstant;
		public string handSideRL;

		public float [] fingerRestricted;
		/// <summary> The FeedbackScript to detect whether or not you're touching something. </summary>
		public SG_FingerFeedback touchScript;

		/// <summary> The amount of extension required to 'unlock' the fingers again. Default set to 5% (0.05) </summary>
		[Range(0, 1)] public float unlockExtension = 0.05f;

		/// <summary> If true, the finger is currently locked. </summary>
		private bool locked = false;
		/// <summary> Finger flexion [0 .. 1] at which the flexion was originally locked. </summary>
		private float lockedFlexion = 0;

		/// <summary> The maximum collider penetration allowed during a soft-lock. </summary>
		private float maxLockDistance = 0;
		/// <summary> The flexion when we reached maxLockDistance. Used to unlock flexion again during a soft lock </summary>
		private float maxLockFlexion = 0;

		public float[] fingerFlexion; //new variable to print all finger values in one array
		public string handComparation;
		DateTime curr;
		DateTime prev;
		TimeSpan delta;
		//--------------------------------------------------------------------------------------------------------------------------
		// Accessors

		/// <summary> Accesses animation state of the finger this script is attached to </summary>
				
		public bool FingerAnimationEnabled
        {
			get
            {
				int fIndex;
				return this.handAnimation != null && GetFingerIndex(out fIndex) ? this.handAnimation.updateFingers[fIndex] : false;
            }
			set
            {
				int fIndex;
				if (this.handAnimation != null && GetFingerIndex(out fIndex)) { handAnimation.updateFingers[fIndex] = value; }
            }
        }


		//--------------------------------------------------------------------------------------------------------------------------
		// Methods / Functions


		/// <summary> Get a valid index to access the correct finger flexion. </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		public bool GetFingerIndex(out int index)
		{
			index = this.touchScript != null ? (int)this.touchScript.handLocation : -1;
			return index > -1 && index < 5; //elliminates unknown and wrist indices
		}


		/// <summary> Returns the flexion relevant to our current finger. </summary>
		/// <param name="currentFlexion"></param>
		/// <returns></returns>
		public bool GetFlexion(out float currentFlexion)
		{
			
			if (handAnimation.Hardware != null)
			{
				handSideRL="SG Left Hand (SG.SG_HapticGlove)"; //<---- Variable for comparation (is it a left hand???)
				float[] flexions;
				int fIndex;

				if (GetFingerIndex(out fIndex) && handAnimation.Hardware.GetNormalizedFlexion(out flexions))
				{
					currentFlexion = flexions[(int)fIndex];
					//if (Input.GetKeyDown("f"))
					//{
						if (String.Equals(handAnimation.Hardware.ToString(), handSideRL)) //Finds which hand is at use: 1.5.1 for left or 1.3.1 for right
						{
							float abductionL, thumbFlexL;
							//Debug.Log("<1.5.1."+flexions[0]+"."+flexions[1]*180+"."+flexions[2]*180+"."+flexions[3]*180+"."+flexions[4]*180);
							if(flexions[0]<0.5)
							{	
								abductionL = flexions[0]*180; //<------ modify for better finger tracking
								thumbFlexL = flexions[0]*50;  //<-------- modify for better finder tracking
								cmdLeft="<1.5.1."+Convert.ToInt16(abductionL)+"."+Convert.ToInt16(thumbFlexL)+"."+Convert.ToInt16(flexions[1]*180)+"."+Convert.ToInt16(flexions[2]*180)+"."+Convert.ToInt16(flexions[3]*180)+"."+Convert.ToInt16(flexions[4]*180)+">";
							}else{	
								abductionL = flexions[0]*180;
								thumbFlexL = flexions[0]*180;
								cmdLeft="<1.5.1."+Convert.ToInt16(abductionL)+"."+Convert.ToInt16(thumbFlexL)+"."+Convert.ToInt16(flexions[1]*180)+"."+Convert.ToInt16(flexions[2]*180)+"."+Convert.ToInt16(flexions[3]*180)+"."+Convert.ToInt16(flexions[4]*180)+">";
							}
						}else{
							float  abductionR, thumbFlexR;
							if(flexions[0]<0.5)
							{	
								abductionR = flexions[0]*180;//<------ modify for better finger tracking
								thumbFlexR = flexions[0]*50; //<-------- modify for better finder tracking
								cmdRight="<1.3.1."+Convert.ToInt16(abductionR)+"."+Convert.ToInt16(thumbFlexR)+"."+Convert.ToInt16(flexions[1]*180)+"."+Convert.ToInt16(flexions[2]*180)+"."+Convert.ToInt16(flexions[3]*180)+"."+Convert.ToInt16(flexions[4]*180)+">";
							}else{	
								abductionR = flexions[0]*180; //<------ modify for better finger tracking
								thumbFlexR = flexions[0]*180; //<-------- modify for better finder tracking
								cmdRight="<1.3.1."+Convert.ToInt16(abductionR)+"."+Convert.ToInt16(thumbFlexR)+"."+Convert.ToInt16(flexions[1]*180)+"."+Convert.ToInt16(flexions[2]*180)+"."+Convert.ToInt16(flexions[3]*180)+"."+Convert.ToInt16(flexions[4]*180)+">";
							}
						}

				//Thread.Sleep(1000);
				
					if (cmdRight!=null)
						{
							// send the chest message
							bytesent = Encoding.ASCII.GetBytes(cmdRight);
							clientManos.Send(bytesent,bytesent.Length);
							Debug.Log("Message "+cmdRight+" sent with success");
						}
					if (cmdLeft!=null)
						{
							// send the chest message
							bytesent = Encoding.ASCII.GetBytes(cmdLeft);
							clientManos.Send(bytesent,bytesent.Length);
							Debug.Log("Message "+cmdLeft+" sent with success");
						}
						
					
					return true;
				}
			}
			currentFlexion = 0;
			return false;

			
		}

		




		/// <summary> Locks the finger in its current flexion.</summary>
		public void HardLockFinger()
        {
			float currFlex;
			if (GetFlexion(out currFlex))
            {
				HardLockFinger(currFlex);
            }
        }


		/// <summary> Locks the finger to a specific flexion. It will only unlock if it extends above currentFlexion by the value set for "unlockextension" </summary>
		private void HardLockFinger(float currentFlexion)
        {
			locked = true;
			lockedFlexion = currentFlexion;
			FingerAnimationEnabled = (false);
			//Debug.Log("Hard Locking " + (this.feedbackScript != null ? this.feedbackScript.handLocation.ToString() : "N\\A") + " at " + currentFlexion);
		}


		/// <summary> Locks the finger at a specific flexion, and limits the maximum collider penetration  </summary>
		/// <param name="currentFlexion"></param>
		/// <param name="maxDisplacement"></param>
		private void SoftLockFinger(float currentFlexion, float maxDisplacement)
        {
			locked = true;
			lockedFlexion = currentFlexion;
			maxLockDistance = maxDisplacement;
			//Debug.Log("Soft Locking " + (this.feedbackScript != null ? this.feedbackScript.handLocation.ToString() : "N\\A") + " at " + currentFlexion + ", with max displ of " + maxDisplacement);
		}



		/// <summary> Returns true if the finger animation can unlock in its current state </summary>
		/// <param name="currentFlexion"></param>
		/// <returns></returns>
		private bool CanUnlock(float currentFlexion)
		{
			return currentFlexion < lockedFlexion - unlockExtension;
		}


		/// <summary> Unlocks the finger so it can move freely once more. </summary>
		public void UnlockFinger()
        {
			locked = false;
			FingerAnimationEnabled = (true);
			maxLockDistance = 0;
		}


		/// <summary> Sets animation state during a soft-lock.  </summary>
		/// <param name="currFlexion"></param>
		private void UpdateSoftLock(float currFlexion)
		{
			if (maxLockDistance > 0)
            {
				if (FingerAnimationEnabled)
                {
					if (touchScript.DistanceInCollider > maxLockDistance)
					{
						FingerAnimationEnabled = (false);
						maxLockFlexion = currFlexion;
					}
				}
				//we're locked because we went too far.
				else if (currFlexion < maxLockFlexion)
                {
					FingerAnimationEnabled = (true);
				}
			}
		}



		/// <summary> Main worker function for this script. Called during FixedUpdate if this script is active. </summary>
		private void CheckFinger()
        {
			if (touchScript != null && handAnimation != null)
            {
				float currFlexion;
				if (GetFlexion(out currFlexion)) //the glove is active
                {
					if (locked)
                    {
						if (!this.touchScript.IsTouching() || this.CanUnlock(currFlexion))
                        {
							//Debug.Log("Unlocking " + (this.feedbackScript != null ? this.feedbackScript.handLocation.ToString() : "N\\A") + " because " 
							//	+ (this.feedbackScript.IsTouching() ? "we've extended enough" : "we're no longer touching"));
							UnlockFinger();
                        }
						else
                        {
							UpdateSoftLock(currFlexion);
						}
                    }
					else //not locked
                    {
						if (!locked && touchScript.IsTouching())
                        {
							float deformDist = touchScript.TouchedDeformScript != null ? touchScript.TouchedDeformScript.maxDisplacement : 0;
							float forceDist = touchScript.TouchedMaterialScript.maxForceDist;
							float breakDist = touchScript.TouchedMaterialScript.breakable ? touchScript.TouchedMaterialScript.yieldDistance : 0;
							float maxMaterialDist = Mathf.Max(Mathf.Max(Mathf.Max(deformDist, forceDist), breakDist), 0.005f ); //5mm break



							if (maxMaterialDist == 0)
							{
								HardLockFinger(currFlexion);
							}
							else
							{
								SoftLockFinger(currFlexion, Mathf.Max(forceDist, maxMaterialDist));
							}
						}
						//update immedeately after if we have a soft lock.
						if (locked)
                        {
							UpdateSoftLock(currFlexion);
                        }
					}
                }
            }
        }


		//--------------------------------------------------------------------------------------------------------------------------
		// Monobehaviour

		private void Start()
        {
			prev = DateTime.Now;
			clientManos.Connect(new IPEndPoint(IPAddress.Parse("10.0.0.20"),9002));
			//handComparation="Animation LayerR";
			// Attempt to retrieve missing references.
			if (this.handAnimation == null && this.touchScript != null && this.touchScript.feedbackScript != null)
            {
				this.handAnimation = this.touchScript.feedbackScript.HandAnimation;
            }
			
        }

		/// <summary> RameUpdate is usually higher than FixedUpdate, allowing us to catch some collisions that we wouldn't ordinarily due to script execution order. </summary>
		private void FixedUpdate()
        {
			
				CheckFinger();
			
        }


	}

}