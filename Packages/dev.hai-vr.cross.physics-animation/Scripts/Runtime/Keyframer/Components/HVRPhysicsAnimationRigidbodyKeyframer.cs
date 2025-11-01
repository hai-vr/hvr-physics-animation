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

using HVR.PhysicsAnimation.Data;
using UnityEngine;
using UnityEngine.Serialization;

namespace HVR.PhysicsAnimation.Runtime
{
    public class HVRPhysicsAnimationRigidbodyPIDKeyframer : MonoBehaviour
    {
        [FormerlySerializedAs("body")] [SerializeField] private Rigidbody bodyOptional;

        [SerializeField] internal HVRPhysicsAnimationRig physicsRig;
        [SerializeField] internal HumanBodyBones humanBodyBone;

        [SerializeField] internal Transform target; // TODO: This is no longer a center of mass

        [SerializeField] internal Animator targetReferenceOptional;

        [SerializeField] internal float proportionalGain = 1000;
        [SerializeField] internal float integralGain = 10;
        [SerializeField] internal float derivativeGain = 40;
        [SerializeField] internal float integralMaximumMagnitude = 10;

        [SerializeField] internal float proportionalTorqueMul = 10f;
        [SerializeField] internal float integralTorqueMul = 1f;
        [SerializeField] internal float derivativeTorqueMul = 1f;
        [SerializeField] internal float integralMaximumMagnitudelTorqueMul = 1f;

        [SerializeField] internal bool compensateGravity = true;
        [SerializeField] internal float forceLimit = 1000f;
        [SerializeField] internal float torqueLimit = 2000f;

        [SerializeField] private bool _debug_clickToResetJoints;
        [SerializeField] private bool _debug_print;
        [SerializeField] private bool _debug_superControl;

        private PIDControllerVector3 _positionPid;
        private PIDControllerAngularVelocity _rotationPid;

        private Vector3 _drawTargetCenterOfMass;
        private Vector3 _drawCurrentCenterOfMass;
        private float _angleDiff;
        private Quaternion _drawCurrentRotation;
        private Quaternion _drawTargetRotation;

        private void Awake()
        {
            _positionPid = new PIDControllerVector3();
            _rotationPid = new PIDControllerAngularVelocity();
        }

        private void OnDisable()
        {
            _positionPid._ResetAsFirstFrame();
            _rotationPid._ResetAsFirstFrame();
        }

        private void FixedUpdate()
        {
            if (bodyOptional == null)
            {
                var rigBone = physicsRig.GetBoneTransform(humanBodyBone);
                bodyOptional = rigBone.GetComponent<Rigidbody>(); // TODO: Have the rigidbody already in the rig
            }

            _positionPid.proportionalGain = proportionalGain;
            _positionPid.integralGain = integralGain;
            _positionPid.derivativeGain = derivativeGain;
            _positionPid.integralMaximumMagnitude = integralMaximumMagnitude;

            _rotationPid.proportionalGain = proportionalGain * proportionalTorqueMul;
            _rotationPid.integralGain = integralGain * integralTorqueMul;
            _rotationPid.derivativeGain = derivativeGain * derivativeTorqueMul;
            _rotationPid.integralMaximumMagnitude = integralMaximumMagnitude * integralMaximumMagnitudelTorqueMul;

            if (_debug_clickToResetJoints)
            {
                _debug_clickToResetJoints = false;
                // TODO: Reset configurable joint
            }

            var currentCenterOfMass = bodyOptional.worldCenterOfMass;
            if (float.IsNaN(currentCenterOfMass.x)) return;

            var actualTarget = targetReferenceOptional != null ? targetReferenceOptional.GetBoneTransform(humanBodyBone) : target;
            var targetCenterOfMass = actualTarget.position + actualTarget.TransformVector(bodyOptional.centerOfMass);

            var force = _positionPid._Update(Time.fixedDeltaTime, currentCenterOfMass, targetCenterOfMass);
            var forceMagnitude = force.magnitude;
            if (forceMagnitude > forceLimit) force = force.normalized * forceLimit;
            bodyOptional.AddForce(force, ForceMode.Acceleration);

            var currentRotation = bodyOptional.transform.rotation;
            var targetRotation = actualTarget.rotation;

            var torque = _rotationPid._Update(Time.fixedDeltaTime, currentRotation, targetRotation);
            var torqueMagnitude = torque.magnitude;
            if (torqueMagnitude > torqueLimit) torque = torque.normalized * torqueLimit;
            bodyOptional.AddTorque(torque, ForceMode.Acceleration);

            _angleDiff = Mathf.Clamp01(Quaternion.Angle(currentRotation, targetRotation) / 30f);

            _drawTargetCenterOfMass = targetCenterOfMass;
            _drawCurrentCenterOfMass = currentCenterOfMass;

            _drawCurrentRotation = currentRotation;
            _drawTargetRotation = targetRotation;

            if (compensateGravity)
            {
                bodyOptional.AddForce(-Physics.gravity, ForceMode.Force);
            }

            Debug__SuperControl();
        }

        private void Debug__SuperControl()
        {
            if (_debug_superControl)
            {
                bodyOptional.position = target.position;
                bodyOptional.rotation = target.rotation;
                bodyOptional.transform.position = target.position;
                bodyOptional.transform.rotation = target.rotation;
            }
        }

        private void Update()
        {
            Debug__SuperControl();

            // dataViz._DrawLine(_drawCurrentCenterOfMass, _drawTargetCenterOfMass, Color.cyan, Color.red, 1f * proportionalGain / 1000);
        }

        private void LateUpdate()
        {
            Debug__SuperControl();
        }
    }
}
