// Copyright 2025 Haï~ (@vr_hai github.com/hai-vr)
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// 
//    http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using HVR.PhysicsAnimation.Data;
using UnityEngine;
using static UnityEngine.HumanBodyBones;

namespace HVR.PhysicsAnimation.Runtime
{
    [DefaultExecutionOrder(-90)] // Must be run after HVR.PhysicsAnimationRemesher
    public class HVRPhysicsAnimationHumanoidRearticulator : MonoBehaviour
    {
        private static readonly HumanBodyBones[] PhysicsRigHbbsInExpectedHierarchyOrder = {
            Hips, Spine, Chest, UpperChest, Neck, Head,
            LeftUpperLeg, LeftLowerLeg, LeftFoot,
            RightUpperLeg, RightLowerLeg, RightFoot,
            LeftShoulder, LeftUpperArm, LeftLowerArm, LeftHand,
            RightShoulder, RightUpperArm, RightLowerArm, RightHand,
        };

        private static readonly HashSet<HumanBodyBones> CollideableBones = new()
        {
            Hips, Spine, Chest, UpperChest, Neck, Head,
        };

        [SerializeField] private bool includeFingers;
        [SerializeField] private Animator humanoidReference;
        [SerializeField] private HVRPhysicsAnimationRig physicsRig; // FIXME: The nullability of this should be replaced. We need the rig

        [SerializeField] private float _debug_estimatedBodyHeight;
        [SerializeField] private float _debug_estimatedBodyMass;
        [SerializeField] private bool _debug_hackTensorsToUniform;

        private readonly List<ConfigurableJoint> _configurableJoints = new();

        private void Awake()
        {
            var availableBones = new List<(HumanBodyBones, Rigidbody)>();

            // Create articulations

            for (HumanBodyBones hbb = Hips; hbb < LastBone; hbb++)
            {
                var isFingerBone = hbb >= LeftThumbProximal && hbb <= RightLittleDistal;
                if (!includeFingers && isFingerBone) continue;

                if (BoneIsInconsequential(hbb)) continue;

                Transform boneTransform;
                if (physicsRig != null) // FIXME: Not expected, migration necessary
                {
                    boneTransform = physicsRig.GetBoneTransform(hbb);
                }
                else
                {
                    boneTransform = humanoidReference.GetBoneTransform(hbb);
                }

                if (boneTransform != null)
                {
                    var body = MakeRigidbody(hbb, boneTransform);
                    availableBones.Add((hbb, body));
                }
            }

            foreach (var configurableJoint in _configurableJoints)
            {
                configurableJoint.connectedBody = configurableJoint.transform.parent.GetComponent<Rigidbody>();
            }

            // Adjust mass

            var totalBodyMass = EstimateTotalBodyMass();

            var totalDistribution = 0f;
            foreach (var availableBone in availableBones)
            {
                totalDistribution += MassDistribution(availableBone.Item1);
            }

            foreach (var availableBone in availableBones)
            {
                var massToApply = (MassDistribution(availableBone.Item1) / totalDistribution) * totalBodyMass;
                availableBone.Item2.mass = massToApply;
            }
        }

        private float EstimateTotalBodyMass()
        {
            // FIXME: We need to resolve this from the humanoid info, not world space.
            var leftFoot = humanoidReference.GetBoneTransform(LeftFoot).position.y;
            var headBase = humanoidReference.GetBoneTransform(Head).position.y;
            var height = (headBase - leftFoot) * 1.25f;
            _debug_estimatedBodyHeight = height;

            // Calculate an estimate for the body mass based on height
            var t = (height - 1.35f) / (1.9f - 1.35f);
            var totalBodyMass = Mathf.LerpUnclamped(30f, 85f, t);
            _debug_estimatedBodyMass = totalBodyMass;

            return totalBodyMass;
        }

        // Provides an arbitrary number which is used for the distribution of mass across the body.
        private int MassDistribution(HumanBodyBones bone)
        {
            return bone switch
            {
                Hips => 100,
                LeftUpperLeg or RightUpperLeg => 80,
                LeftLowerLeg or RightLowerLeg => 70,
                LeftFoot or RightFoot => 50,
                Spine => 85,
                Chest => 70,
                UpperChest => 60,
                Neck => 50,
                Head => 30,
                LeftShoulder or RightShoulder => 50,
                LeftUpperArm or RightUpperArm => 40,
                LeftLowerArm or RightLowerArm => 40,
                LeftHand or RightHand => 40,
                LeftToes or RightToes => 10,
                LeftEye or RightEye => 1,
                Jaw => 1,
                LeftThumbProximal
                    or LeftIndexProximal
                    or LeftMiddleProximal
                    or LeftRingProximal
                    or LeftLittleProximal
                    or RightThumbProximal
                    or RightIndexProximal
                    or RightMiddleProximal
                    or RightRingProximal
                    or RightLittleProximal => 4,
                LeftThumbIntermediate
                    or LeftIndexIntermediate
                    or LeftMiddleIntermediate
                    or LeftRingIntermediate
                    or LeftLittleIntermediate
                    or RightThumbIntermediate
                    or RightIndexIntermediate
                    or RightMiddleIntermediate
                    or RightRingIntermediate
                    or RightLittleIntermediate => 3,
                LeftThumbDistal
                    or LeftIndexDistal
                    or LeftMiddleDistal
                    or LeftRingDistal
                    or LeftLittleDistal
                    or RightThumbDistal
                    or RightIndexDistal
                    or RightMiddleDistal
                    or RightRingDistal
                    or RightLittleDistal => 2,
                LastBone => 1,
                _ => throw new ArgumentOutOfRangeException(nameof(bone), bone, null)
            };
        }

        private static bool BoneIsInconsequential(HumanBodyBones bone)
        {
            return bone is LeftEye
                or RightEye
                or Jaw
                or LeftShoulder
                or RightShoulder
                or UpperChest
                or LeftToes
                or RightToes;
        }

        private Rigidbody MakeRigidbody(HumanBodyBones hbb, Transform boneTransform)
        {
            var go = boneTransform.gameObject;
            var body = go.AddComponent<Rigidbody>();
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            body.useGravity = false; // The keyframe controller should set this to true.
            if (!CollideableBones.Contains(hbb))
            {
                body.excludeLayers = int.MaxValue;
            }

            // FIXME: The inertia tensor for the spine and the foot on Wolfram are wrong.
            // body.automaticInertiaTensor = false;
            // body.inertiaTensor = Vector3.one * 0.17f;
            var automaticTensorValue = body.inertiaTensor;
            body.automaticInertiaTensor = false;
            if (false)
            {
                var minimumTensorComponent = 0.05f;
                // var minimumTensorComponent = automaticTensorValue.magnitude * 0.5f;
                var adjustedTensor = new Vector3(
                    Mathf.Max(minimumTensorComponent, automaticTensorValue.x),
                    Mathf.Max(minimumTensorComponent, automaticTensorValue.y),
                    Mathf.Max(minimumTensorComponent, automaticTensorValue.z)
                );
                // FIXME: this is a hack
                if (_debug_hackTensorsToUniform)
                {
                    adjustedTensor = Vector3.one * 0.17f;
                }
                body.inertiaTensor = adjustedTensor;
            }
            else
            {
                body.inertiaTensor = Vector3.one * 0.17f;
            }
#if UNITY_EDITOR
            var applier = body.gameObject.AddComponent<HVRPhysicsAnimationInertiaTensorEditorApplier>();
            applier.multiplier = 0.17f;
            applier.baseInertiaTensor = Vector3.one;
            applier.Apply();
#endif

            if (hbb != Hips)
            {
                var joint = go.AddComponent<ConfigurableJoint>();
                joint.xMotion = ConfigurableJointMotion.Locked;
                joint.yMotion = ConfigurableJointMotion.Locked;
                joint.zMotion = ConfigurableJointMotion.Locked;
                joint.angularXMotion = ConfigurableJointMotion.Limited;
                joint.angularYMotion = ConfigurableJointMotion.Limited;
                joint.angularZMotion = ConfigurableJointMotion.Limited;
                var limit = 90;
                joint.lowAngularXLimit = new SoftJointLimit { limit = -limit, bounciness = 0, contactDistance = 0};
                joint.highAngularXLimit = new SoftJointLimit { limit = limit, bounciness = 0, contactDistance = 0 };
                joint.angularYLimit = new SoftJointLimit { limit = limit, bounciness = 0, contactDistance = 0 };
                joint.angularZLimit = new SoftJointLimit { limit = limit, bounciness = 0, contactDistance = 0 };

                // joint.slerpDrive = new JointDrive { positionSpring = 0f, positionDamper = 100f, maximumForce = Mathf.Infinity };
                // hack: PositionDamper 0 is an attempt to fix arms flailing
                // TODO: Driven, see readme.md, when not-driven then positionDamper should probably be 0.
                if (false)
                {
                    // DEMO FOR FLAILING ARMS ONLY
                    var isArmMember = hbb is LeftUpperArm or RightUpperArm or LeftLowerArm or RightLowerArm or LeftHand or RightHand;
                    var positionSpring = isArmMember ? 10f : 0f;
                    var positionDamper = isArmMember ? positionSpring : 100f;
                    joint.slerpDrive = new JointDrive { positionSpring = positionSpring, positionDamper = positionDamper, maximumForce = Mathf.Infinity };
                }
                else
                {
                    joint.slerpDrive = new JointDrive { positionSpring = 0f, positionDamper = 100f, maximumForce = Mathf.Infinity };
                }
                joint.rotationDriveMode = RotationDriveMode.Slerp;

                // Junk
                // joint.angularXLimitSpring = new SoftJointLimitSpring { damper = 75f, spring = 2000f };
                // joint.angularYZLimitSpring = new SoftJointLimitSpring { damper = 75f, spring = 2000f };
                // joint.configuredInWorldSpace = false;
                // joint.autoConfigureConnectedAnchor = false;

                _configurableJoints.Add(joint);
            }

            return body;
        }
    }
}
