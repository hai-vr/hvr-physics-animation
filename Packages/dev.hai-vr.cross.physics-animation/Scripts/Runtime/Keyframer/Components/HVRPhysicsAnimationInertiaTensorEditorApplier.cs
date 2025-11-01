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
    public class HVRPhysicsAnimationInertiaTensorEditorApplier : MonoBehaviour
    {
        public Vector3 baseInertiaTensor = Vector3.one;
        public float multiplier = 1f;

        public float minMultiplier = 0.001f;

        private void OnValidate()
        {
            Apply();
            // OnValidate ONLY WORKS IN EDITOR.
        }

        public void Apply()
        {
            var rb = GetComponent<Rigidbody>();
            rb.inertiaTensor = baseInertiaTensor * Mathf.Clamp(multiplier, minMultiplier, float.MaxValue);
        }
    }
}
