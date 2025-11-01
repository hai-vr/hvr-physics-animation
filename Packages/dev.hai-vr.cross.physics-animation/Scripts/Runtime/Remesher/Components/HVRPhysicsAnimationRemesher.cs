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
using Unity.Collections;
using UnityEngine;

namespace HVR.PhysicsAnimation.Runtime
{
    /// Given SkinnedMeshRenderers, this splits the mesh into several meshes where each mesh represents a bone,
    /// and adds a convex hull MeshCollider to each bone. All the mesh simplification concerns are handled by
    /// what is already built-in the MeshCollider, not us.
    [DefaultExecutionOrder(-100)] // Needs to be run before HVR.PhysicsAnimationHumanoidReacticulator
    public class HVRPhysicsAnimationRemesher : MonoBehaviour
    {
        private const float BoneWeightAcceptanceThreshold = 0.4f;

        // TODO: Should we use FastMidphase? does it matter?
        private const MeshColliderCookingOptions Cooking = MeshColliderCookingOptions.EnableMeshCleaning
                                                           | MeshColliderCookingOptions.WeldColocatedVertices
                                                           | MeshColliderCookingOptions.CookForFasterSimulation
                                                           | MeshColliderCookingOptions.UseFastMidphase;

        [SerializeField] private SkinnedMeshRenderer[] sources; // UGC Rule.
        [SerializeField] private HVRRemesherRigidbodyPhysics rigidbodyPhysics = HVRRemesherRigidbodyPhysics.CreateCollidersOnSeparateGameObjectsWithRigidbodies;
        [SerializeField] private bool excludeFingerBones = false;
        [SerializeField] private Animator humanoidReference; // must be non-null if excludeFingerBones is true.
        [SerializeField] private PhysicsMaterial physicsMaterial;

        [SerializeField] private HVRPhysicsAnimationRig physicsRig;
        [SerializeField] private Transform newRigRoot; // Only used for CreateCollidersOnSeparateRig

        private void Awake()
        {
            var boneExclusions = new HashSet<Transform>();
            if (excludeFingerBones)
            {
                if (humanoidReference == null)
                {
                    enabled = false;
                    throw new ArgumentException("If you are excluding finger bones, then we need to know the humanoid.");
                }

                for (HumanBodyBones bone = HumanBodyBones.LeftThumbProximal; bone <= HumanBodyBones.RightLittleDistal; bone++)
                {
                    var boneTransform = humanoidReference.GetBoneTransform(bone);
                    if (boneTransform != null)
                    {
                        boneExclusions.Add(boneTransform);
                    }
                }
            }

            if (rigidbodyPhysics == HVRRemesherRigidbodyPhysics.CreateCollidersOnSeparateRig)
            {
                var hips = RecreateExistingBone(HumanBodyBones.Hips);
                hips.gameObject.SetActive(false);
                hips.transform.SetParent(newRigRoot, true);

                var spine = RecreateExistingBone(HumanBodyBones.Spine);
                var chest = RecreateExistingBone(HumanBodyBones.Chest);
                // TODO: Recreate upper chest
                var neck = RecreateExistingBone(HumanBodyBones.Neck);
                var head = RecreateExistingBone(HumanBodyBones.Head);

                var upperArmLeft = RecreateExistingBone(HumanBodyBones.LeftUpperArm);
                var lowerArmLeft = RecreateExistingBone(HumanBodyBones.LeftLowerArm);
                var handLeft = RecreateExistingBone(HumanBodyBones.LeftHand);

                var upperArmRight = RecreateExistingBone(HumanBodyBones.RightUpperArm);
                var lowerArmRight = RecreateExistingBone(HumanBodyBones.RightLowerArm);
                var handRight = RecreateExistingBone(HumanBodyBones.RightHand);

                var upperLegLeft = RecreateExistingBone(HumanBodyBones.LeftUpperLeg);
                var lowerLegLeft = RecreateExistingBone(HumanBodyBones.LeftLowerLeg);
                var footLeft = RecreateExistingBone(HumanBodyBones.LeftFoot);

                var upperLegRight = RecreateExistingBone(HumanBodyBones.RightUpperLeg);
                var lowerLegRight = RecreateExistingBone(HumanBodyBones.RightLowerLeg);
                var footRight = RecreateExistingBone(HumanBodyBones.RightFoot);

                spine.SetParent(hips, true);
                chest.SetParent(spine, true);
                var chestConnector = chest;
                // TODO: Upper chest

                neck.SetParent(chestConnector, true);
                head.SetParent(neck, true);

                upperArmLeft.SetParent(chestConnector, true);
                lowerArmLeft.SetParent(upperArmLeft, true);
                handLeft.SetParent(lowerArmLeft, true);

                upperArmRight.SetParent(chestConnector, true);
                lowerArmRight.SetParent(upperArmRight, true);
                handRight.SetParent(lowerArmRight, true);

                upperLegLeft.SetParent(hips, true);
                lowerLegLeft.SetParent(upperLegLeft, true);
                footLeft.SetParent(lowerLegLeft, true);

                upperLegRight.SetParent(hips, true);
                lowerLegRight.SetParent(upperLegRight, true);
                footRight.SetParent(lowerLegRight, true);
            }

            var allColliders = new List<Collider>();
            foreach (var skinnedMeshRenderer in sources)
            {
                if (null != skinnedMeshRenderer)
                {
                    var generatedColliders = ManipulateSmr(skinnedMeshRenderer, boneExclusions);

                    foreach (Collider generatedCollider in generatedColliders)
                    {
                        generatedCollider.excludeLayers = 1 << LayerMask.NameToLayer("Player");
                    }

                    allColliders.AddRange(generatedColliders);
                }
            }

            if (rigidbodyPhysics == HVRRemesherRigidbodyPhysics.CreateCollidersOnSeparateRig)
            {
                var hips = newRigRoot.GetChild(0).GetComponent<MeshCollider>();
                IgnoreRecurse(hips);

                var spine = physicsRig.GetBoneTransform(HumanBodyBones.Spine).GetComponent<MeshCollider>();
                var chest = physicsRig.GetBoneTransform(HumanBodyBones.Chest).GetComponent<MeshCollider>();
                Physics.IgnoreCollision(
                    chest,
                    hips
                );

                Physics.IgnoreCollision(
                    spine,
                    physicsRig.GetBoneTransform(HumanBodyBones.LeftUpperLeg).GetComponent<MeshCollider>()
                );
                Physics.IgnoreCollision(
                    spine,
                    physicsRig.GetBoneTransform(HumanBodyBones.RightUpperLeg).GetComponent<MeshCollider>()
                );
                Physics.IgnoreCollision(
                    chest,
                    physicsRig.GetBoneTransform(HumanBodyBones.LeftUpperLeg).GetComponent<MeshCollider>()
                );
                Physics.IgnoreCollision(
                    chest,
                    physicsRig.GetBoneTransform(HumanBodyBones.RightUpperLeg).GetComponent<MeshCollider>()
                );
            }

            // PhysicsIgnoreIntraCollisions(allColliders);
        }

        private void IgnoreRecurse(MeshCollider current)
        {
            foreach (Transform child in current.transform)
            {
                var childCollider = child.GetComponent<MeshCollider>();
                Physics.IgnoreCollision(current, childCollider);
                IgnoreRecurse(childCollider);
            }
        }

        private Transform RecreateExistingBone(HumanBodyBones bone)
        {
            var existingBone = humanoidReference.GetBoneTransform(bone);
            var result = new GameObject
            {
                transform = { position = existingBone.position, rotation = existingBone.rotation },
                name = bone.ToString()
            }.transform;

            physicsRig.SetBoneTransform(bone, result);

            return result;
        }

        private List<Collider> ManipulateSmr(SkinnedMeshRenderer skinnedMeshRenderer, HashSet<Transform> boneExclusions)
        {
            var originalMesh = skinnedMeshRenderer.sharedMesh;

            var vertexCount = originalMesh.vertexCount;
            var boneCountPerVertex = originalMesh.GetBonesPerVertex();
            var allBoneWeights = originalMesh.GetAllBoneWeights();
            var vertexIdToStartingIndexInsideBoneWeightsArray = CalculateVertexIdToStartingIndexInsideBoneWeightsArray(boneCountPerVertex);

            var totalBoneCount = originalMesh.bindposes.Length;

            var boneIndexToMajorlyVertexIds = new List<List<int>>();
            for (var i = 0; i < totalBoneCount; i++)
            {
                boneIndexToMajorlyVertexIds.Add(new List<int>());
            }

            // Triage each vertexId into the bones it belongs to.

            for (var vertexId = 0; vertexId < vertexCount; vertexId++)
            {
                if (boneCountPerVertex[vertexId] > 0) // Guarantees that thisVertexWeights is non-empty
                {
                    var thisVertexWeights = ReadInputBoneWeightsAsNewList(vertexId, boneCountPerVertex, vertexIdToStartingIndexInsideBoneWeightsArray, allBoneWeights);

                    var mostWeighted = thisVertexWeights[0];
                    boneIndexToMajorlyVertexIds[mostWeighted.boneIndex].Add(vertexId);

                    // Notice how this starts at 1. We always want the heaviest bone to be assigned, even if it's below threshold.
                    for (var i = 1; i < thisVertexWeights.Count; i++)
                    {
                        var currentWeight = thisVertexWeights[i];
                        if (currentWeight.weight > BoneWeightAcceptanceThreshold)
                        {
                            boneIndexToMajorlyVertexIds[currentWeight.boneIndex].Add(vertexId);
                        }
                    }
                }
            }

            // Rebuild the mesh

            var smrBones = skinnedMeshRenderer.bones;

            var generatedMeshes = new List<Mesh>();
            var whichBoneIndexForThatGeneratedMesh = new List<int>();

            var originalBindposes = originalMesh.bindposes;
            var originalVertices = originalMesh.vertices;
            var originalTriangles = originalMesh.triangles;
            for (var boneIndex = 0; boneIndex < totalBoneCount; boneIndex++)
            {
                var smrBoneNullable = smrBones[boneIndex];
                if (smrBoneNullable != null && boneExclusions.Contains(smrBoneNullable)) // FIXME: If it's nullable, we probably shouldn't create the collider in the first place
                {
                    continue;
                }

                var majorlyVertexIds = boneIndexToMajorlyVertexIds[boneIndex];

                if (majorlyVertexIds.Count > 0)
                {
                    var bindposeForThisBone = originalBindposes[boneIndex];

                    var verticesForThisBone = new Vector3[majorlyVertexIds.Count];
                    for (var index = 0; index < majorlyVertexIds.Count; index++)
                    {
                        var vertexId = majorlyVertexIds[index];
                        var v3 = originalVertices[vertexId];
                        verticesForThisBone[index] = bindposeForThisBone * new Vector4(v3.x, v3.y, v3.z, 1);
                    }

                    if (verticesForThisBone.Length >= 3)
                    {
                        var trianglesForThisBone = ReconstructTriangles(majorlyVertexIds, originalTriangles);

                        var thereIsAtLeastOneTriangle = trianglesForThisBone.Length >= 3;
                        if (thereIsAtLeastOneTriangle)
                        {
                            var meshForThisBone = new Mesh();

                            meshForThisBone.vertices = verticesForThisBone;
                            meshForThisBone.triangles = trianglesForThisBone;

                            generatedMeshes.Add(meshForThisBone);
                            whichBoneIndexForThatGeneratedMesh.Add(boneIndex);

                            Physics.BakeMesh(meshForThisBone.GetInstanceID(), true, Cooking);
                        }
                    }
                }
            }

            // Rig building

            var createdColliders = new List<Collider>();

            if (rigidbodyPhysics != HVRRemesherRigidbodyPhysics.CreateCollidersOnSeparateRig)
            {
                for (var index = 0; index < generatedMeshes.Count; index++)
                {
                    var generatedMesh = generatedMeshes[index];
                    var boneIndex = whichBoneIndexForThatGeneratedMesh[index];

                    var smrBoneNullable = smrBones[boneIndex];

                    if (rigidbodyPhysics is HVRRemesherRigidbodyPhysics.None or HVRRemesherRigidbodyPhysics.CreateCollidersOnSeparateGameObjectsWithRigidbodies)
                    {
                        var go = new GameObject
                        {
                            name = $"{skinnedMeshRenderer.name}_MeshCollider_Bone{boneIndex:000}",
                            transform =
                            {
                                // TODO: When adding the head, if it's a local avatar, then add it to the same system that the
                                // shadow clone head transform uses to avoid first-person shrinking.
                                parent = smrBoneNullable != null ? smrBoneNullable.transform : null, // FIXME: null parent is a mistake!
                                localPosition = Vector3.zero,
                                localRotation = Quaternion.identity,
                                localScale = skinnedMeshRenderer.transform.localScale // FIXME: Likely incorrect
                            }
                        };
                        go.SetActive(false); // Ensure that components initialize in one go, after we have defined them.

                        var ourCollider = go.AddComponent<MeshCollider>();
                        ourCollider.sharedMaterial = physicsMaterial;
                        ourCollider.convex = true;
                        ourCollider.cookingOptions = Cooking;
                        ourCollider.sharedMesh = generatedMesh;
                        createdColliders.Add(ourCollider);

                        if (rigidbodyPhysics == HVRRemesherRigidbodyPhysics.CreateCollidersOnSeparateGameObjectsWithRigidbodies)
                        {
                            var previousRigidbodyNullable = go.GetComponent<Rigidbody>();
                            var ourRigidbody = previousRigidbodyNullable != null ? previousRigidbodyNullable : go.AddComponent<Rigidbody>();
                            ourRigidbody.isKinematic = true;
                            ourRigidbody.mass = 1f;
                            ourRigidbody.automaticCenterOfMass = true;
                            ourRigidbody.useGravity = false;
                        }

                        go.SetActive(true);
                    }
                    else if (rigidbodyPhysics is HVRRemesherRigidbodyPhysics.CreateCollidersOnBonesWithRigidbodies or HVRRemesherRigidbodyPhysics.CreateCollidersOnBones)
                    {
                        if (smrBoneNullable != null)
                        {
                            var go = smrBoneNullable.gameObject;

                            Collider ourCollider2;
                            if (true)
                            {
                                var ourCollider = go.AddComponent<MeshCollider>();
                                ourCollider.sharedMaterial = physicsMaterial;
                                ourCollider.convex = true;
                                ourCollider.cookingOptions = Cooking;
                                ourCollider.sharedMesh = generatedMesh;
                                ourCollider2 = ourCollider;
                            }
                            else
                            {
                                var ourCollider = go.AddComponent<SphereCollider>();
                                ourCollider.radius = 0.05f;
                                ourCollider2 = ourCollider;
                            }

                            createdColliders.Add(ourCollider2);
                        }
                    }
                }
            }
            else
            {
                var applicableHbbs = new[]
                {
                    HumanBodyBones.Hips,
                    HumanBodyBones.Spine,
                    HumanBodyBones.Chest,
                    HumanBodyBones.Neck,
                    HumanBodyBones.Head,
                    HumanBodyBones.LeftUpperArm,
                    HumanBodyBones.LeftLowerArm,
                    HumanBodyBones.LeftHand,
                    HumanBodyBones.RightUpperArm,
                    HumanBodyBones.RightLowerArm,
                    HumanBodyBones.RightHand,
                    HumanBodyBones.LeftUpperLeg,
                    HumanBodyBones.LeftLowerLeg,
                    HumanBodyBones.LeftFoot,
                    HumanBodyBones.RightUpperLeg,
                    HumanBodyBones.RightLowerLeg,
                    HumanBodyBones.RightFoot
                };
                var matchedSmrBones = new List<Transform>();
                foreach (var hbb in applicableHbbs)
                {
                    matchedSmrBones.Add(humanoidReference.GetBoneTransform(hbb));
                }

                for (var index = 0; index < generatedMeshes.Count; index++)
                {
                    var generatedMesh = generatedMeshes[index];
                    var boneIndex = whichBoneIndexForThatGeneratedMesh[index];

                    var smrBone = smrBones[boneIndex];
                    if (smrBone != null)
                    {
                        var matchedSmrBoneHbbIndex = matchedSmrBones.IndexOf(smrBone);
                        if (matchedSmrBoneHbbIndex != -1)
                        {
                            var go = physicsRig.GetBoneTransform(applicableHbbs[matchedSmrBoneHbbIndex]).gameObject;

                            if (true)
                            {
                                // HACK: Avoid creating two mesh collders on the neck
                                // if (go.GetComponent<MeshCollider>() == null)
                                {
                                    var ourCollider = go.AddComponent<MeshCollider>();
                                    ourCollider.sharedMaterial = physicsMaterial;
                                    ourCollider.convex = true;
                                    ourCollider.cookingOptions = Cooking;
                                    ourCollider.sharedMesh = generatedMesh;
                                    createdColliders.Add(ourCollider);
                                }
                            }
                            else
                            {
                                var ourCollider = go.AddComponent<SphereCollider>();
                                ourCollider.radius = 0.05f;
                                createdColliders.Add(ourCollider);
                            }

                        }
                    }
                }

                newRigRoot.GetChild(0).gameObject.SetActive(true);
            }


            if (rigidbodyPhysics == HVRRemesherRigidbodyPhysics.CreateCollidersOnBonesWithRigidbodies)
            {
                foreach (var thatCollider in createdColliders)
                {
                    var go = thatCollider.gameObject;
                    var previousRigidbodyNullable = go.GetComponent<Rigidbody>();
                    var ourRigidbody = previousRigidbodyNullable != null ? previousRigidbodyNullable : go.AddComponent<Rigidbody>();
                    ourRigidbody.isKinematic = true;
                    ourRigidbody.mass = 1f;
                    ourRigidbody.automaticCenterOfMass = true;
                    ourRigidbody.useGravity = false;
                }
            }

            return createdColliders;
        }

        private static void PhysicsIgnoreIntraCollisions(List<Collider> colliders)
        {
            for (var i1 = 0; i1 < colliders.Count - 1; i1++)
            {
                var a = colliders[i1];
                for (var i2 = 1; i2 < colliders.Count; i2++)
                {
                    var b = colliders[i2];
                    Physics.IgnoreCollision(a, b);
                }
            }
        }

        private static int[] ReconstructTriangles(List<int> majorlyVertexIds, int[] originalTriangles)
        {
            var reconstructedTriangles = new List<int>();

            var keepThoseVertices = new HashSet<int>(majorlyVertexIds);

            for (var indexWithinTriangles = 0; indexWithinTriangles < originalTriangles.Length; indexWithinTriangles += 3)
            {
                var vertexIdForA = originalTriangles[indexWithinTriangles];
                var vertexIdForB = originalTriangles[indexWithinTriangles + 1];
                var vertexIdForC = originalTriangles[indexWithinTriangles + 2];

                // (Note: The default Basis avatar is painted too weird, so the arms and lower body won't pass this condition.)
                if (keepThoseVertices.Contains(vertexIdForA)
                    && keepThoseVertices.Contains(vertexIdForB)
                    && keepThoseVertices.Contains(vertexIdForC))
                {
                    // The index of the vertexId inside the majorlyVertexIds list is the new vertexId of our new mesh.
                    var regeneratedVertexIdForA = majorlyVertexIds.IndexOf(vertexIdForA);
                    var regeneratedVertexIdForB = majorlyVertexIds.IndexOf(vertexIdForB);
                    var regeneratedVertexIdForC = majorlyVertexIds.IndexOf(vertexIdForC);

                    reconstructedTriangles.Add(regeneratedVertexIdForA);
                    reconstructedTriangles.Add(regeneratedVertexIdForB);
                    reconstructedTriangles.Add(regeneratedVertexIdForC);
                }
            }

            return reconstructedTriangles.ToArray();
        }

        private static int[] CalculateVertexIdToStartingIndexInsideBoneWeightsArray(NativeArray<byte> boneCountPerVertex)
        {
            var startingIndices = new List<int>();
            var anchor = 0;
            foreach (var boneCountForThatVertex in boneCountPerVertex)
            {
                startingIndices.Add(anchor);
                anchor += boneCountForThatVertex;
            }

            var idToStartingIndexInsideBoneWeightsArray = startingIndices.ToArray();
            return idToStartingIndexInsideBoneWeightsArray;
        }

        private static List<BoneWeight1> ReadInputBoneWeightsAsNewList(int vertexId, NativeArray<byte> boneCountPerVertex, int[] vertexIdToStartingIndexInsideBoneWeightsArray, NativeArray<BoneWeight1> allBoneWeights)
        {
            var startingIndex = vertexIdToStartingIndexInsideBoneWeightsArray[vertexId];

            var boneWeight1s = new List<BoneWeight1>();
            for (var offset = 0; offset < boneCountPerVertex[vertexId]; offset++)
            {
                var currentBoneWeight = allBoneWeights[startingIndex + offset];
                boneWeight1s.Add(currentBoneWeight);
            }

            return boneWeight1s;
        }
    }

    public enum HVRRemesherRigidbodyPhysics
    {
        None,
        CreateCollidersOnSeparateGameObjectsWithRigidbodies,
        CreateCollidersOnBonesWithRigidbodies,
        CreateCollidersOnBones,
        CreateCollidersOnSeparateRig
    }
}
