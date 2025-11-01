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

using System.Collections.Generic;
using UnityEngine;

namespace HVR.PhysicsAnimation.Data
{
    public class HVRPhysicsAnimationRig : MonoBehaviour
    {
        private readonly Dictionary<HumanBodyBones, Transform> _boneToTransform = new();

        // TODO: Expose RigidBody directly

        /// Returns null if the bone doesn't exist.
        public Transform GetBoneTransform(HumanBodyBones bone)
        {
            return _boneToTransform.GetValueOrDefault(bone);
        }

        /// Setting a bone to null is allowed.
        public void SetBoneTransform(HumanBodyBones bone, Transform value)
        {
            if (value == null) _boneToTransform.Remove(bone);
            else _boneToTransform[bone] = value;
        }
    }
}
