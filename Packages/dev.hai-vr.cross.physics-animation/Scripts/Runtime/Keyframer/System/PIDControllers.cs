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

using UnityEngine;

namespace HVR.PhysicsAnimation.Runtime
{
    internal class PIDControllerVector3
    {
        public float proportionalGain;
        public float integralGain;
        public float derivativeGain;

        public float integralMaximumMagnitude = 1f;

        private Vector3 _previous;
        private Vector3 _integration;

        // Whenever the object spawns or teleports, we need skip the first derivative in order to prevent a sudden jump.
        private bool _skipFirstDerivative = true;

        public void _ResetAsFirstFrame()
        {
            _skipFirstDerivative = true;
            _previous = Vector3.zero;
            _integration = Vector3.zero;
        }

        public Vector3 _Update(float fixedDeltaTime, Vector3 current, Vector3 target)
        {
            var error = target - current;
            var proportionalValue = proportionalGain * error;

            var change = (current - _previous) / fixedDeltaTime;
            _previous = current;

            Vector3 derivativeValue;
            if (_skipFirstDerivative)
            {
                derivativeValue = Vector3.zero;
                _skipFirstDerivative = false;
            }
            else
            {
                derivativeValue = derivativeGain * -change;
            }

            _integration += error * fixedDeltaTime;
            if (_integration.magnitude > integralMaximumMagnitude)
            {
                _integration = _integration.normalized * integralMaximumMagnitude;
            }
            var integralValue = integralGain * _integration;

            return proportionalValue + derivativeValue + integralValue;
        }
    }

    internal class PIDControllerAngularVelocity
    {
        public float proportionalGain;
        public float integralGain;
        public float derivativeGain;

        public float integralMaximumMagnitude = 1f;

        private Quaternion _previous;
        private Vector3 _integration;

        // Whenever the object spawns or teleports, we need skip the first derivative in order to prevent a sudden jump.
        private bool _skipFirstDerivative = true;

        public void _ResetAsFirstFrame()
        {
            _skipFirstDerivative = true;
            _previous = Quaternion.identity;
            _integration = Vector3.zero;
        }

        public Vector3 _Update(float fixedDeltaTime, Quaternion current, Quaternion target)
        {
            // var error = target - current;
            var errorQuat = target * Quaternion.Inverse(current);
            var error = MovementToAngularVelocityBase(errorQuat);
            var proportionalValue = proportionalGain * error;

            // var change = (current - _previous) / fixedDeltaTime;
            var change = MovementToAngularVelocityBase(current * Quaternion.Inverse(_previous)) / fixedDeltaTime;
            _previous = current;

            Vector3 derivativeValue;
            if (_skipFirstDerivative)
            {
                derivativeValue = Vector3.zero;
                _skipFirstDerivative = false;
            }
            else
            {
                derivativeValue = derivativeGain * -change;
            }

            _integration += error * fixedDeltaTime;
            if (_integration.magnitude > integralMaximumMagnitude)
            {
                _integration = _integration.normalized * integralMaximumMagnitude;
            }
            var integralValue = integralGain * _integration;

            return proportionalValue + derivativeValue + integralValue;
        }

        private Vector3 MovementToAngularVelocityBase(Quaternion movementRequired)
        {
            movementRequired.ToAngleAxis(out var signedDegrees, out var axis);
            if (signedDegrees > 180f) signedDegrees -= 360f; // NOT SURE: Try to fix a weird singularity in the angular velocity
            var angularVelBase = axis * (signedDegrees * Mathf.Deg2Rad);
            return angularVelBase;
        }
    }
}
