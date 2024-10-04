struct physx_PxAllocatorCallback_Pod;
struct physx_PxErrorCallback_Pod;
struct physx_PxAssertHandler_Pod;
struct physx_PxInputStream_Pod;
struct physx_PxInputData_Pod;
struct physx_PxOutputStream_Pod;
struct physx_PxVec2_Pod;
struct physx_PxVec3_Pod;
struct physx_PxVec4_Pod;
struct physx_PxQuat_Pod;
struct physx_PxMat33_Pod;
struct physx_PxMat34_Pod;
struct physx_PxMat44_Pod;
struct physx_PxTransform_Pod;
struct physx_PxPlane_Pod;
struct physx_PxBounds3_Pod;
struct physx_PxAllocatorCallback_Pod {
    void* vtable_;
};
struct physx_PxAssertHandler_Pod {
    void* vtable_;
};
struct physx_PxAllocationListener_Pod;
struct physx_PxFoundation_Pod {
    void* vtable_;
};
struct physx_PxProfilerCallback_Pod;
struct physx_PxAllocator_Pod {
    char structgen_pad0[1];
};
struct physx_PxRawAllocator_Pod {
    char structgen_pad0[1];
};
struct physx_PxVirtualAllocatorCallback_Pod {
    void* vtable_;
};
struct physx_PxVirtualAllocator_Pod {
    char structgen_pad0[16];
};
struct physx_PxUserAllocated_Pod {
    char structgen_pad0[1];
};
union physx_PxTempAllocatorChunk_Pod {
    physx_PxTempAllocatorChunk_Pod* mNext;
    uint32_t mIndex;
    uint8_t mPad[16];
};
struct physx_PxTempAllocator_Pod {
    char structgen_pad0[1];
};
struct physx_PxLogTwo_Pod;
struct physx_PxUnConst_Pod;
struct physx_PxBitAndByte_Pod {
    char structgen_pad0[1];
};
struct physx_PxBitMap_Pod {
    char structgen_pad0[16];
};
struct physx_PxVec3_Pod {
    ctype='float' x;
    ctype='float' y;
    ctype='float' z;
};
struct physx_PxVec3Padded_Pod {
    ctype='float' x;
    ctype='float' y;
    ctype='float' z;
    ctype='uint32_t' padding;
};
struct physx_PxQuat_Pod {
    ctype='float' x;
    ctype='float' y;
    ctype='float' z;
    ctype='float' w;
};
struct physx_PxTransform_Pod {
    ctype='physx_PxQuat_Pod' q;
    ctype='physx_PxVec3_Pod' p;
};
struct physx_PxTransformPadded_Pod {
    ctype='physx_PxTransform_Pod' transform;
    ctype='uint32_t' padding;
};
struct physx_PxMat33_Pod {
    ctype='physx_PxVec3_Pod' column0;
    ctype='physx_PxVec3_Pod' column1;
    ctype='physx_PxVec3_Pod' column2;
};
struct physx_PxBounds3_Pod {
    ctype='physx_PxVec3_Pod' minimum;
    ctype='physx_PxVec3_Pod' maximum;
};
struct physx_PxErrorCallback_Pod {
    void* vtable_;
};
struct physx_PxAllocationListener_Pod {
    void* vtable_;
};
struct physx_PxBroadcastingAllocator_Pod {
    char structgen_pad0[176];
};
struct physx_PxBroadcastingErrorCallback_Pod {
    char structgen_pad0[160];
};
struct physx_PxHash_Pod;
struct physx_PxInputStream_Pod {
    void* vtable_;
};
struct physx_PxInputData_Pod {
    void* vtable_;
};
struct physx_PxOutputStream_Pod {
    void* vtable_;
};
struct physx_PxVec4_Pod {
    ctype='float' x;
    ctype='float' y;
    ctype='float' z;
    ctype='float' w;
};
struct physx_PxMat44_Pod {
    ctype='physx_PxVec4_Pod' column0;
    ctype='physx_PxVec4_Pod' column1;
    ctype='physx_PxVec4_Pod' column2;
    ctype='physx_PxVec4_Pod' column3;
};
struct physx_PxPlane_Pod {
    ctype='physx_PxVec3_Pod' n;
    ctype='float' d;
};
struct physx_Interpolation_Pod {
    char structgen_pad0[1];
};
struct physx_PxMutexImpl_Pod {
    char structgen_pad0[1];
};
struct physx_PxReadWriteLock_Pod {
    char structgen_pad0[8];
};
struct physx_PxProfilerCallback_Pod {
    void* vtable_;
};
struct physx_PxProfileScoped_Pod {
    ctype='physx_PxProfilerCallback_Pod*' mCallback;
    ctype='char const*' mEventName;
    ctype='void*' mProfilerData;
    ctype='uint64_t' mContextId;
    ctype='bool' mDetached;
    char structgen_pad0[7];
};
struct physx_PxSListEntry_Pod {
    char structgen_pad0[16];
};
struct physx_PxSListImpl_Pod {
    char structgen_pad0[1];
};
struct physx_PxSyncImpl_Pod {
    char structgen_pad0[1];
};
struct physx_PxRunnable_Pod {
    void* vtable_;
};
struct physx_PxCounterFrequencyToTensOfNanos_Pod {
    ctype='uint64_t' mNumerator;
    ctype='uint64_t' mDenominator;
};
struct physx_PxTime_Pod {
    char structgen_pad0[8];
};
struct physx_PxVec2_Pod {
    ctype='float' x;
    ctype='float' y;
};
struct physx_PxStridedData_Pod {
    ctype='uint32_t' stride;
    char structgen_pad0[4];
    ctype='void const*' data;
};
struct physx_PxBoundedData_Pod {
    ctype='uint32_t' stride;
    char structgen_pad0[4];
    ctype='void const*' data;
    ctype='uint32_t' count;
    char structgen_pad1[4];
};
struct physx_PxDebugPoint_Pod {
    ctype='physx_PxVec3_Pod' pos;
    ctype='uint32_t' color;
};
struct physx_PxDebugLine_Pod {
    ctype='physx_PxVec3_Pod' pos0;
    ctype='uint32_t' color0;
    ctype='physx_PxVec3_Pod' pos1;
    ctype='uint32_t' color1;
};
struct physx_PxDebugTriangle_Pod {
    ctype='physx_PxVec3_Pod' pos0;
    ctype='uint32_t' color0;
    ctype='physx_PxVec3_Pod' pos1;
    ctype='uint32_t' color1;
    ctype='physx_PxVec3_Pod' pos2;
    ctype='uint32_t' color2;
};
struct physx_PxDebugText_Pod {
    ctype='physx_PxVec3_Pod' position;
    ctype='float' size;
    ctype='uint32_t' color;
    char structgen_pad0[4];
    ctype='char const*' string;
};
struct physx_PxRenderBuffer_Pod {
    void* vtable_;
};
struct physx_PxBase_Pod;
struct physx_PxSerializationContext_Pod;
struct physx_PxRepXSerializer_Pod;
struct physx_PxSerializer_Pod;
struct physx_PxPhysics_Pod;
struct physx_PxCollection_Pod;
struct physx_PxProcessPxBaseCallback_Pod {
    void* vtable_;
};
struct physx_PxSerializationContext_Pod {
    void* vtable_;
};
struct physx_PxDeserializationContext_Pod {
    char structgen_pad0[16];
};
struct physx_PxSerializationRegistry_Pod {
    void* vtable_;
};
struct physx_PxCollection_Pod {
    void* vtable_;
};
struct physx_PxTypeInfo_Pod;
struct physx_PxMaterial_Pod;
struct physx_PxFEMSoftBodyMaterial_Pod;
struct physx_PxFEMClothMaterial_Pod;
struct physx_PxPBDMaterial_Pod;
struct physx_PxFLIPMaterial_Pod;
struct physx_PxMPMMaterial_Pod;
struct physx_PxCustomMaterial_Pod;
struct physx_PxConvexMesh_Pod;
struct physx_PxTriangleMesh_Pod;
struct physx_PxBVH33TriangleMesh_Pod;
struct physx_PxBVH34TriangleMesh_Pod;
struct physx_PxTetrahedronMesh_Pod;
struct physx_PxHeightField_Pod;
struct physx_PxActor_Pod;
struct physx_PxRigidActor_Pod;
struct physx_PxRigidBody_Pod;
struct physx_PxRigidDynamic_Pod;
struct physx_PxRigidStatic_Pod;
struct physx_PxArticulationLink_Pod;
struct physx_PxArticulationJointReducedCoordinate_Pod;
struct physx_PxArticulationReducedCoordinate_Pod;
struct physx_PxAggregate_Pod;
struct physx_PxConstraint_Pod;
struct physx_PxShape_Pod;
struct physx_PxPruningStructure_Pod;
struct physx_PxParticleSystem_Pod;
struct physx_PxPBDParticleSystem_Pod;
struct physx_PxFLIPParticleSystem_Pod;
struct physx_PxMPMParticleSystem_Pod;
struct physx_PxCustomParticleSystem_Pod;
struct physx_PxSoftBody_Pod;
struct physx_PxFEMCloth_Pod;
struct physx_PxHairSystem_Pod;
struct physx_PxParticleBuffer_Pod;
struct physx_PxParticleAndDiffuseBuffer_Pod;
struct physx_PxParticleClothBuffer_Pod;
struct physx_PxParticleRigidBuffer_Pod;
struct physx_PxBase_Pod {
    char structgen_pad0[16];
};
struct physx_PxRefCounted_Pod {
    char structgen_pad0[16];
};
struct physx_PxTolerancesScale_Pod {
    ctype='float' length;
    ctype='float' speed;
};
struct physx_PxStringTable_Pod {
    void* vtable_;
};
struct physx_PxSerializer_Pod {
    void* vtable_;
};
struct physx_PxMetaDataEntry_Pod {
    ctype='char const*' type;
    ctype='char const*' name;
    ctype='uint32_t' offset;
    ctype='uint32_t' size;
    ctype='uint32_t' count;
    ctype='uint32_t' offsetSize;
    ctype='uint32_t' flags;
    ctype='uint32_t' alignment;
};
struct physx_PxInsertionCallback_Pod {
    void* vtable_;
};
struct physx_PxBaseTask_Pod;
struct physx_PxTask_Pod;
struct physx_PxLightCpuTask_Pod;
struct physx_PxCpuDispatcher_Pod;
struct physx_PxTaskManager_Pod {
    void* vtable_;
};
struct physx_PxCpuDispatcher_Pod {
    void* vtable_;
};
struct physx_PxBaseTask_Pod {
    char structgen_pad0[24];
};
struct physx_PxTask_Pod {
    char structgen_pad0[32];
};
struct physx_PxLightCpuTask_Pod {
    char structgen_pad0[40];
};
struct physx_PxGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
};
struct physx_PxBoxGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='physx_PxVec3_Pod' halfExtents;
};
struct physx_PxBVHRaycastCallback_Pod {
    void* vtable_;
};
struct physx_PxBVHOverlapCallback_Pod {
    void* vtable_;
};
struct physx_PxBVHTraversalCallback_Pod {
    void* vtable_;
};
struct physx_PxBVH_Pod {
    char structgen_pad0[16];
};
struct physx_PxGeomIndexPair_Pod;
struct physx_PxCapsuleGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='float' radius;
    ctype='float' halfHeight;
};
struct physx_PxHullPolygon_Pod {
    float mPlane[4];
    ctype='uint16_t' mNbVerts;
    ctype='uint16_t' mIndexBase;
};
struct physx_PxConvexMesh_Pod {
    char structgen_pad0[16];
};
struct physx_PxMeshScale_Pod {
    ctype='physx_PxVec3_Pod' scale;
    ctype='physx_PxQuat_Pod' rotation;
};
struct physx_PxConvexMeshGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='physx_PxMeshScale_Pod' scale;
    char structgen_pad1[4];
    ctype='physx_PxConvexMesh_Pod*' convexMesh;
    ctype='uint8_t' meshFlags;
    char structgen_pad2[7];
};
struct physx_PxSphereGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='float' radius;
};
struct physx_PxPlaneGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
};
struct physx_PxTriangleMeshGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='physx_PxMeshScale_Pod' scale;
    ctype='uint8_t' meshFlags;
    char structgen_pad1[3];
    ctype='physx_PxTriangleMesh_Pod*' triangleMesh;
};
struct physx_PxHeightFieldGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='physx_PxHeightField_Pod*' heightField;
    ctype='float' heightScale;
    ctype='float' rowScale;
    ctype='float' columnScale;
    ctype='uint8_t' heightFieldFlags;
    char structgen_pad1[3];
};
struct physx_PxParticleSystemGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='int32_t' mSolverType;
};
struct physx_PxHairSystemGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
};
struct physx_PxTetrahedronMeshGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='physx_PxTetrahedronMesh_Pod*' tetrahedronMesh;
};
struct physx_PxQueryHit_Pod {
    ctype='uint32_t' faceIndex;
};
struct physx_PxLocationHit_Pod {
    ctype='uint32_t' faceIndex;
    ctype='uint16_t' flags;
    char structgen_pad0[2];
    ctype='physx_PxVec3_Pod' position;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' distance;
};
struct physx_PxGeomRaycastHit_Pod {
    ctype='uint32_t' faceIndex;
    ctype='uint16_t' flags;
    char structgen_pad0[2];
    ctype='physx_PxVec3_Pod' position;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' distance;
    ctype='float' u;
    ctype='float' v;
};
struct physx_PxGeomOverlapHit_Pod {
    ctype='uint32_t' faceIndex;
};
struct physx_PxGeomSweepHit_Pod {
    ctype='uint32_t' faceIndex;
    ctype='uint16_t' flags;
    char structgen_pad0[2];
    ctype='physx_PxVec3_Pod' position;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' distance;
};
struct physx_PxGeomIndexPair_Pod {
    ctype='uint32_t' id0;
    ctype='uint32_t' id1;
};
struct physx_PxQueryThreadContext_Pod {
    char structgen_pad0[1];
};
struct physx_PxContactBuffer_Pod;
struct physx_PxRenderOutput_Pod;
struct physx_PxMassProperties_Pod;
struct physx_PxCustomGeometryType_Pod {
    char structgen_pad0[4];
};
struct physx_PxCustomGeometryCallbacks_Pod {
    void* vtable_;
};
struct physx_PxCustomGeometry_Pod {
    char structgen_pad0[4];
    ctype='float' mTypePadding;
    ctype='physx_PxCustomGeometryCallbacks_Pod*' callbacks;
};
struct physx_PxGeometryHolder_Pod {
    char structgen_pad0[56];
};
struct physx_PxGeometryQuery_Pod {
    char structgen_pad0[1];
};
struct physx_PxHeightFieldSample_Pod {
    ctype='int16_t' height;
    ctype='physx_PxBitAndByte_Pod' materialIndex0;
    ctype='physx_PxBitAndByte_Pod' materialIndex1;
};
struct physx_PxHeightFieldDesc_Pod;
struct physx_PxHeightField_Pod {
    char structgen_pad0[16];
};
struct physx_PxHeightFieldDesc_Pod {
    ctype='uint32_t' nbRows;
    ctype='uint32_t' nbColumns;
    ctype='int32_t' format;
    char structgen_pad0[4];
    ctype='physx_PxStridedData_Pod' samples;
    ctype='float' convexEdgeThreshold;
    ctype='uint16_t' flags;
    char structgen_pad1[2];
};
struct physx_PxTriangle_Pod;
struct physx_PxMeshQuery_Pod {
    char structgen_pad0[1];
};
struct physx_PxSimpleTriangleMesh_Pod {
    ctype='physx_PxBoundedData_Pod' points;
    ctype='physx_PxBoundedData_Pod' triangles;
    ctype='uint16_t' flags;
    char structgen_pad0[6];
};
struct physx_PxTriangle_Pod {
    physx_PxVec3_Pod verts[3];
};
struct physx_PxTrianglePadded_Pod {
    physx_PxVec3_Pod verts[3];
    ctype='uint32_t' padding;
};
struct physx_PxTriangleMesh_Pod {
    char structgen_pad0[16];
};
struct physx_PxBVH34TriangleMesh_Pod {
    char structgen_pad0[16];
};
struct physx_PxTetrahedron_Pod {
    physx_PxVec3_Pod verts[4];
};
struct physx_PxSoftBodyAuxData_Pod {
    char structgen_pad0[16];
};
struct physx_PxTetrahedronMesh_Pod {
    char structgen_pad0[16];
};
struct physx_PxSoftBodyMesh_Pod {
    char structgen_pad0[16];
};
struct physx_PxCollisionMeshMappingData_Pod {
    char structgen_pad0[8];
};
struct physx_PxSoftBodyCollisionData_Pod {
    char structgen_pad0[1];
};
struct physx_PxTetrahedronMeshData_Pod {
    char structgen_pad0[1];
};
struct physx_PxSoftBodySimulationData_Pod {
    char structgen_pad0[1];
};
struct physx_PxCollisionTetrahedronMeshData_Pod {
    char structgen_pad0[8];
};
struct physx_PxSimulationTetrahedronMeshData_Pod {
    char structgen_pad0[8];
};
struct physx_PxScene_Pod;
struct physx_PxActor_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxAggregate_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxSpringModifiers_Pod {
    ctype='float' stiffness;
    ctype='float' damping;
    char structgen_pad0[8];
};
struct physx_PxRestitutionModifiers_Pod {
    ctype='float' restitution;
    ctype='float' velocityThreshold;
    char structgen_pad0[8];
};
union physx_Px1DConstraintMods_Pod {
    physx_PxSpringModifiers_Pod spring;
    physx_PxRestitutionModifiers_Pod bounce;
};
struct physx_Px1DConstraint_Pod {
    ctype='physx_PxVec3_Pod' linear0;
    ctype='float' geometricError;
    ctype='physx_PxVec3_Pod' angular0;
    ctype='float' velocityTarget;
    ctype='physx_PxVec3_Pod' linear1;
    ctype='float' minImpulse;
    ctype='physx_PxVec3_Pod' angular1;
    ctype='float' maxImpulse;
    ctype='physx_Px1DConstraintMods_Pod' mods;
    ctype='float' forInternalUse;
    ctype='uint16_t' flags;
    ctype='uint16_t' solveHint;
    char structgen_pad0[8];
};
struct physx_PxConstraintInvMassScale_Pod {
    ctype='float' linear0;
    ctype='float' angular0;
    ctype='float' linear1;
    ctype='float' angular1;
};
struct physx_PxConstraintVisualizer_Pod {
    void* vtable_;
};
struct physx_PxConstraintConnector_Pod {
    void* vtable_;
};
struct physx_PxContactPoint_Pod {
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' separation;
    ctype='physx_PxVec3_Pod' point;
    ctype='float' maxImpulse;
    ctype='physx_PxVec3_Pod' targetVel;
    ctype='float' staticFriction;
    ctype='uint8_t' materialFlags;
    char structgen_pad0[3];
    ctype='uint32_t' internalFaceIndex1;
    ctype='float' dynamicFriction;
    ctype='float' restitution;
    ctype='float' damping;
    char structgen_pad1[12];
};
struct physx_PxTGSSolverBodyVel_Pod;
struct physx_PxSolverBody_Pod {
    ctype='physx_PxVec3_Pod' linearVelocity;
    ctype='uint16_t' maxSolverNormalProgress;
    ctype='uint16_t' maxSolverFrictionProgress;
    ctype='physx_PxVec3_Pod' angularState;
    ctype='uint32_t' solverProgress;
};
struct physx_PxSolverBodyData_Pod {
    ctype='physx_PxVec3_Pod' linearVelocity;
    ctype='float' invMass;
    ctype='physx_PxVec3_Pod' angularVelocity;
    ctype='float' reportThreshold;
    ctype='physx_PxMat33_Pod' sqrtInvInertia;
    ctype='float' penBiasClamp;
    ctype='uint32_t' nodeIndex;
    ctype='float' maxContactImpulse;
    ctype='physx_PxTransform_Pod' body2World;
    ctype='uint16_t' pad;
    char structgen_pad0[2];
};
struct physx_PxConstraintBatchHeader_Pod {
    ctype='uint32_t' startIndex;
    ctype='uint16_t' stride;
    ctype='uint16_t' constraintType;
};
struct physx_PxSolverConstraintDesc_Pod {
    char structgen_pad0[16];
    ctype='uint32_t' bodyADataIndex;
    ctype='uint32_t' bodyBDataIndex;
    ctype='uint32_t' linkIndexA;
    ctype='uint32_t' linkIndexB;
    ctype='uint8_t*' constraint;
    ctype='void*' writeBack;
    ctype='uint16_t' progressA;
    ctype='uint16_t' progressB;
    ctype='uint16_t' constraintLengthOver16;
    uint8_t padding[10];
};
struct physx_PxSolverConstraintPrepDescBase_Pod {
    ctype='physx_PxConstraintInvMassScale_Pod' invMassScales;
    ctype='physx_PxSolverConstraintDesc_Pod*' desc;
    ctype='physx_PxSolverBody_Pod const*' body0;
    ctype='physx_PxSolverBody_Pod const*' body1;
    ctype='physx_PxSolverBodyData_Pod const*' data0;
    ctype='physx_PxSolverBodyData_Pod const*' data1;
    ctype='physx_PxTransform_Pod' bodyFrame0;
    ctype='physx_PxTransform_Pod' bodyFrame1;
    ctype='int32_t' bodyState0;
    ctype='int32_t' bodyState1;
    char structgen_pad0[8];
};
struct physx_PxSolverConstraintPrepDesc_Pod {
    ctype='physx_PxConstraintInvMassScale_Pod' invMassScales;
    ctype='physx_PxSolverConstraintDesc_Pod*' desc;
    ctype='physx_PxSolverBody_Pod const*' body0;
    ctype='physx_PxSolverBody_Pod const*' body1;
    ctype='physx_PxSolverBodyData_Pod const*' data0;
    ctype='physx_PxSolverBodyData_Pod const*' data1;
    ctype='physx_PxTransform_Pod' bodyFrame0;
    ctype='physx_PxTransform_Pod' bodyFrame1;
    ctype='int32_t' bodyState0;
    ctype='int32_t' bodyState1;
    char structgen_pad0[8];
    ctype='physx_Px1DConstraint_Pod*' rows;
    ctype='uint32_t' numRows;
    ctype='float' linBreakForce;
    ctype='float' angBreakForce;
    ctype='float' minResponseThreshold;
    ctype='void*' writeback;
    ctype='bool' disablePreprocessing;
    ctype='bool' improvedSlerp;
    ctype='bool' driveLimitsAreForces;
    ctype='bool' extendedLimits;
    ctype='bool' disableConstraint;
    char structgen_pad1[3];
    ctype='physx_PxVec3Padded_Pod' body0WorldOffset;
    char structgen_pad2[8];
};
struct physx_PxSolverContactDesc_Pod {
    ctype='physx_PxConstraintInvMassScale_Pod' invMassScales;
    ctype='physx_PxSolverConstraintDesc_Pod*' desc;
    ctype='physx_PxSolverBody_Pod const*' body0;
    ctype='physx_PxSolverBody_Pod const*' body1;
    ctype='physx_PxSolverBodyData_Pod const*' data0;
    ctype='physx_PxSolverBodyData_Pod const*' data1;
    ctype='physx_PxTransform_Pod' bodyFrame0;
    ctype='physx_PxTransform_Pod' bodyFrame1;
    ctype='int32_t' bodyState0;
    ctype='int32_t' bodyState1;
    ctype='void*' shapeInteraction;
    ctype='physx_PxContactPoint_Pod*' contacts;
    ctype='uint32_t' numContacts;
    ctype='bool' hasMaxImpulse;
    ctype='bool' disableStrongFriction;
    ctype='bool' hasForceThresholds;
    char structgen_pad0[1];
    ctype='float' restDistance;
    ctype='float' maxCCDSeparation;
    ctype='uint8_t*' frictionPtr;
    ctype='uint8_t' frictionCount;
    char structgen_pad1[7];
    ctype='float*' contactForces;
    ctype='uint32_t' startFrictionPatchIndex;
    ctype='uint32_t' numFrictionPatches;
    ctype='uint32_t' startContactPatchIndex;
    ctype='uint16_t' numContactPatches;
    ctype='uint16_t' axisConstraintCount;
    ctype='float' offsetSlop;
    char structgen_pad2[12];
};
struct physx_PxConstraintAllocator_Pod {
    void* vtable_;
};
struct physx_PxArticulationLimit_Pod {
    ctype='float' low;
    ctype='float' high;
};
struct physx_PxArticulationDrive_Pod {
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' maxForce;
    ctype='int32_t' driveType;
};
struct physx_PxTGSSolverBodyVel_Pod {
    ctype='physx_PxVec3_Pod' linearVelocity;
    ctype='uint16_t' nbStaticInteractions;
    ctype='uint16_t' maxDynamicPartition;
    ctype='physx_PxVec3_Pod' angularVelocity;
    ctype='uint32_t' partitionMask;
    ctype='physx_PxVec3_Pod' deltaAngDt;
    ctype='float' maxAngVel;
    ctype='physx_PxVec3_Pod' deltaLinDt;
    ctype='uint16_t' lockFlags;
    ctype='bool' isKinematic;
    ctype='uint8_t' pad;
};
struct physx_PxTGSSolverBodyTxInertia_Pod {
    ctype='physx_PxTransform_Pod' deltaBody2World;
    ctype='physx_PxMat33_Pod' sqrtInvInertia;
};
struct physx_PxTGSSolverBodyData_Pod {
    ctype='physx_PxVec3_Pod' originalLinearVelocity;
    ctype='float' maxContactImpulse;
    ctype='physx_PxVec3_Pod' originalAngularVelocity;
    ctype='float' penBiasClamp;
    ctype='float' invMass;
    ctype='uint32_t' nodeIndex;
    ctype='float' reportThreshold;
    ctype='uint32_t' pad;
};
struct physx_PxTGSSolverConstraintPrepDescBase_Pod {
    ctype='physx_PxConstraintInvMassScale_Pod' invMassScales;
    ctype='physx_PxSolverConstraintDesc_Pod*' desc;
    ctype='physx_PxTGSSolverBodyVel_Pod const*' body0;
    ctype='physx_PxTGSSolverBodyVel_Pod const*' body1;
    ctype='physx_PxTGSSolverBodyTxInertia_Pod const*' body0TxI;
    ctype='physx_PxTGSSolverBodyTxInertia_Pod const*' body1TxI;
    ctype='physx_PxTGSSolverBodyData_Pod const*' bodyData0;
    ctype='physx_PxTGSSolverBodyData_Pod const*' bodyData1;
    ctype='physx_PxTransform_Pod' bodyFrame0;
    ctype='physx_PxTransform_Pod' bodyFrame1;
    ctype='int32_t' bodyState0;
    ctype='int32_t' bodyState1;
    char structgen_pad0[8];
};
struct physx_PxTGSSolverConstraintPrepDesc_Pod {
    ctype='physx_PxConstraintInvMassScale_Pod' invMassScales;
    ctype='physx_PxSolverConstraintDesc_Pod*' desc;
    ctype='physx_PxTGSSolverBodyVel_Pod const*' body0;
    ctype='physx_PxTGSSolverBodyVel_Pod const*' body1;
    ctype='physx_PxTGSSolverBodyTxInertia_Pod const*' body0TxI;
    ctype='physx_PxTGSSolverBodyTxInertia_Pod const*' body1TxI;
    ctype='physx_PxTGSSolverBodyData_Pod const*' bodyData0;
    ctype='physx_PxTGSSolverBodyData_Pod const*' bodyData1;
    ctype='physx_PxTransform_Pod' bodyFrame0;
    ctype='physx_PxTransform_Pod' bodyFrame1;
    ctype='int32_t' bodyState0;
    ctype='int32_t' bodyState1;
    ctype='physx_Px1DConstraint_Pod*' rows;
    ctype='uint32_t' numRows;
    ctype='float' linBreakForce;
    ctype='float' angBreakForce;
    ctype='float' minResponseThreshold;
    ctype='void*' writeback;
    ctype='bool' disablePreprocessing;
    ctype='bool' improvedSlerp;
    ctype='bool' driveLimitsAreForces;
    ctype='bool' extendedLimits;
    ctype='bool' disableConstraint;
    char structgen_pad0[3];
    ctype='physx_PxVec3Padded_Pod' body0WorldOffset;
    ctype='physx_PxVec3Padded_Pod' cA2w;
    ctype='physx_PxVec3Padded_Pod' cB2w;
};
struct physx_PxTGSSolverContactDesc_Pod {
    ctype='physx_PxConstraintInvMassScale_Pod' invMassScales;
    ctype='physx_PxSolverConstraintDesc_Pod*' desc;
    ctype='physx_PxTGSSolverBodyVel_Pod const*' body0;
    ctype='physx_PxTGSSolverBodyVel_Pod const*' body1;
    ctype='physx_PxTGSSolverBodyTxInertia_Pod const*' body0TxI;
    ctype='physx_PxTGSSolverBodyTxInertia_Pod const*' body1TxI;
    ctype='physx_PxTGSSolverBodyData_Pod const*' bodyData0;
    ctype='physx_PxTGSSolverBodyData_Pod const*' bodyData1;
    ctype='physx_PxTransform_Pod' bodyFrame0;
    ctype='physx_PxTransform_Pod' bodyFrame1;
    ctype='int32_t' bodyState0;
    ctype='int32_t' bodyState1;
    ctype='void*' shapeInteraction;
    ctype='physx_PxContactPoint_Pod*' contacts;
    ctype='uint32_t' numContacts;
    ctype='bool' hasMaxImpulse;
    ctype='bool' disableStrongFriction;
    ctype='bool' hasForceThresholds;
    char structgen_pad0[1];
    ctype='float' restDistance;
    ctype='float' maxCCDSeparation;
    ctype='uint8_t*' frictionPtr;
    ctype='uint8_t' frictionCount;
    char structgen_pad1[7];
    ctype='float*' contactForces;
    ctype='uint32_t' startFrictionPatchIndex;
    ctype='uint32_t' numFrictionPatches;
    ctype='uint32_t' startContactPatchIndex;
    ctype='uint16_t' numContactPatches;
    ctype='uint16_t' axisConstraintCount;
    ctype='float' maxImpulse;
    ctype='float' torsionalPatchRadius;
    ctype='float' minTorsionalPatchRadius;
    ctype='float' offsetSlop;
};
struct physx_PxArticulationSpatialTendon_Pod;
struct physx_PxArticulationFixedTendon_Pod;
struct physx_PxArticulationTendonLimit_Pod {
    ctype='float' lowLimit;
    ctype='float' highLimit;
};
struct physx_PxArticulationAttachment_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationTendonJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationTendon_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationSpatialTendon_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationFixedTendon_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxSpatialForce_Pod {
    ctype='physx_PxVec3_Pod' force;
    ctype='float' pad0;
    ctype='physx_PxVec3_Pod' torque;
    ctype='float' pad1;
};
struct physx_PxSpatialVelocity_Pod {
    ctype='physx_PxVec3_Pod' linear;
    ctype='float' pad0;
    ctype='physx_PxVec3_Pod' angular;
    ctype='float' pad1;
};
struct physx_PxArticulationRootLinkData_Pod {
    ctype='physx_PxTransform_Pod' transform;
    ctype='physx_PxVec3_Pod' worldLinVel;
    ctype='physx_PxVec3_Pod' worldAngVel;
    ctype='physx_PxVec3_Pod' worldLinAccel;
    ctype='physx_PxVec3_Pod' worldAngAccel;
};
struct physx_PxArticulationCache_Pod {
    ctype='physx_PxSpatialForce_Pod*' externalForces;
    ctype='float*' denseJacobian;
    ctype='float*' massMatrix;
    ctype='float*' jointVelocity;
    ctype='float*' jointAcceleration;
    ctype='float*' jointPosition;
    ctype='float*' jointForce;
    ctype='float*' jointSolverForces;
    ctype='physx_PxSpatialVelocity_Pod*' linkVelocity;
    ctype='physx_PxSpatialVelocity_Pod*' linkAcceleration;
    ctype='physx_PxArticulationRootLinkData_Pod*' rootLinkData;
    ctype='physx_PxSpatialForce_Pod*' sensorForces;
    ctype='float*' coefficientMatrix;
    ctype='float*' lambda;
    ctype='void*' scratchMemory;
    ctype='void*' scratchAllocator;
    ctype='uint32_t' version;
    char structgen_pad0[4];
};
struct physx_PxArticulationSensor_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationReducedCoordinate_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationJointReducedCoordinate_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxFilterData_Pod;
struct physx_PxBaseMaterial_Pod;
struct physx_PxShape_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxRigidActor_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxNodeIndex_Pod {
    char structgen_pad0[8];
};
struct physx_PxRigidBody_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxArticulationLink_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxConeLimitedConstraint_Pod {
    ctype='physx_PxVec3_Pod' mAxis;
    ctype='float' mAngle;
    ctype='float' mLowLimit;
    ctype='float' mHighLimit;
};
struct physx_PxConeLimitParams_Pod {
    ctype='physx_PxVec4_Pod' lowHighLimits;
    ctype='physx_PxVec4_Pod' axisAngle;
};
struct physx_PxConstraintShaderTable_Pod {
    ctype='void *' solverPrep;
    char structgen_pad0[8];
    ctype='void *' visualize;
    ctype='int32_t' flag;
    char structgen_pad1[4];
};
struct physx_PxConstraint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxMassModificationProps_Pod {
    ctype='float' mInvMassScale0;
    ctype='float' mInvInertiaScale0;
    ctype='float' mInvMassScale1;
    ctype='float' mInvInertiaScale1;
};
struct physx_PxContactPatch_Pod {
    ctype='physx_PxMassModificationProps_Pod' mMassModification;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' restitution;
    ctype='float' dynamicFriction;
    ctype='float' staticFriction;
    ctype='float' damping;
    ctype='uint16_t' startContactIndex;
    ctype='uint8_t' nbContacts;
    ctype='uint8_t' materialFlags;
    ctype='uint16_t' internalFlags;
    ctype='uint16_t' materialIndex0;
    ctype='uint16_t' materialIndex1;
    uint16_t pad[5];
};
struct physx_PxContact_Pod {
    ctype='physx_PxVec3_Pod' contact;
    ctype='float' separation;
};
struct physx_PxExtendedContact_Pod {
    ctype='physx_PxVec3_Pod' contact;
    ctype='float' separation;
    ctype='physx_PxVec3_Pod' targetVelocity;
    ctype='float' maxImpulse;
};
struct physx_PxModifiableContact_Pod {
    ctype='physx_PxVec3_Pod' contact;
    ctype='float' separation;
    ctype='physx_PxVec3_Pod' targetVelocity;
    ctype='float' maxImpulse;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' restitution;
    ctype='uint32_t' materialFlags;
    ctype='uint16_t' materialIndex0;
    ctype='uint16_t' materialIndex1;
    ctype='float' staticFriction;
    ctype='float' dynamicFriction;
};
struct physx_PxContactStreamIterator_Pod {
    ctype='physx_PxVec3_Pod' zero;
    char structgen_pad0[4];
    ctype='physx_PxContactPatch_Pod const*' patch;
    ctype='physx_PxContact_Pod const*' contact;
    ctype='uint32_t const*' faceIndice;
    ctype='uint32_t' totalPatches;
    ctype='uint32_t' totalContacts;
    ctype='uint32_t' nextContactIndex;
    ctype='uint32_t' nextPatchIndex;
    ctype='uint32_t' contactPatchHeaderSize;
    ctype='uint32_t' contactPointSize;
    ctype='int32_t' mStreamFormat;
    ctype='uint32_t' forceNoResponse;
    ctype='bool' pointStepped;
    char structgen_pad1[3];
    ctype='uint32_t' hasFaceIndices;
};
struct physx_PxGpuContactPair_Pod {
    ctype='uint8_t*' contactPatches;
    ctype='uint8_t*' contactPoints;
    ctype='float*' contactForces;
    ctype='uint32_t' transformCacheRef0;
    ctype='uint32_t' transformCacheRef1;
    ctype='physx_PxNodeIndex_Pod' nodeIndex0;
    ctype='physx_PxNodeIndex_Pod' nodeIndex1;
    ctype='physx_PxActor_Pod*' actor0;
    ctype='physx_PxActor_Pod*' actor1;
    ctype='uint16_t' nbContacts;
    ctype='uint16_t' nbPatches;
    char structgen_pad0[4];
};
struct physx_PxContactSet_Pod {
    char structgen_pad0[16];
};
struct physx_PxContactModifyPair_Pod {
    physx_PxRigidActor_Pod const* actor[2];
    physx_PxShape_Pod const* shape[2];
    physx_PxTransform_Pod transform[2];
    ctype='physx_PxContactSet_Pod' contacts;
};
struct physx_PxContactModifyCallback_Pod {
    void* vtable_;
};
struct physx_PxCCDContactModifyCallback_Pod {
    void* vtable_;
};
struct physx_PxDeletionListener_Pod {
    void* vtable_;
};
struct physx_PxBaseMaterial_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxFEMMaterial_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxFilterData_Pod {
    ctype='uint32_t' word0;
    ctype='uint32_t' word1;
    ctype='uint32_t' word2;
    ctype='uint32_t' word3;
};
struct physx_PxSimulationFilterCallback_Pod {
    void* vtable_;
};
struct physx_PxParticleRigidFilterPair_Pod {
    ctype='uint64_t' mID0;
    ctype='uint64_t' mID1;
};
struct physx_PxLockedData_Pod {
    void* vtable_;
};
struct physx_PxMaterial_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxGpuParticleBufferIndexPair_Pod {
    ctype='uint32_t' systemIndex;
    ctype='uint32_t' bufferIndex;
};
struct physx_PxCudaContextManager_Pod;
struct physx_PxParticleRigidAttachment_Pod;
struct physx_PxParticleVolume_Pod {
    ctype='physx_PxBounds3_Pod' bound;
    ctype='uint32_t' particleIndicesOffset;
    ctype='uint32_t' numParticles;
};
struct physx_PxDiffuseParticleParams_Pod {
    ctype='float' threshold;
    ctype='float' lifetime;
    ctype='float' airDrag;
    ctype='float' bubbleDrag;
    ctype='float' buoyancy;
    ctype='float' kineticEnergyWeight;
    ctype='float' pressureWeight;
    ctype='float' divergenceWeight;
    ctype='float' collisionDecay;
    ctype='bool' useAccurateVelocity;
    char structgen_pad0[3];
};
struct physx_PxParticleSpring_Pod {
    ctype='uint32_t' ind0;
    ctype='uint32_t' ind1;
    ctype='float' length;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' pad;
};
struct physx_PxParticleMaterial_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxSceneDesc_Pod;
struct physx_PxPvd_Pod;
struct physx_PxOmniPvd_Pod;
struct physx_PxPhysics_Pod {
    void* vtable_;
};
struct physx_PxActorShape_Pod {
    ctype='physx_PxRigidActor_Pod*' actor;
    ctype='physx_PxShape_Pod*' shape;
};
struct physx_PxRaycastHit_Pod {
    ctype='uint32_t' faceIndex;
    ctype='uint16_t' flags;
    char structgen_pad0[2];
    ctype='physx_PxVec3_Pod' position;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' distance;
    ctype='float' u;
    ctype='float' v;
    char structgen_pad1[4];
    ctype='physx_PxRigidActor_Pod*' actor;
    ctype='physx_PxShape_Pod*' shape;
};
struct physx_PxOverlapHit_Pod {
    ctype='uint32_t' faceIndex;
    char structgen_pad0[4];
    ctype='physx_PxRigidActor_Pod*' actor;
    ctype='physx_PxShape_Pod*' shape;
};
struct physx_PxSweepHit_Pod {
    ctype='uint32_t' faceIndex;
    ctype='uint16_t' flags;
    char structgen_pad0[2];
    ctype='physx_PxVec3_Pod' position;
    ctype='physx_PxVec3_Pod' normal;
    ctype='float' distance;
    char structgen_pad1[4];
    ctype='physx_PxRigidActor_Pod*' actor;
    ctype='physx_PxShape_Pod*' shape;
};
struct physx_PxRaycastCallback_Pod {
    char structgen_pad0[8];
    ctype='physx_PxRaycastHit_Pod' block;
    ctype='bool' hasBlock;
    char structgen_pad1[7];
    ctype='physx_PxRaycastHit_Pod*' touches;
    ctype='uint32_t' maxNbTouches;
    ctype='uint32_t' nbTouches;
};
struct physx_PxOverlapCallback_Pod {
    char structgen_pad0[8];
    ctype='physx_PxOverlapHit_Pod' block;
    ctype='bool' hasBlock;
    char structgen_pad1[7];
    ctype='physx_PxOverlapHit_Pod*' touches;
    ctype='uint32_t' maxNbTouches;
    ctype='uint32_t' nbTouches;
};
struct physx_PxSweepCallback_Pod {
    char structgen_pad0[8];
    ctype='physx_PxSweepHit_Pod' block;
    ctype='bool' hasBlock;
    char structgen_pad1[7];
    ctype='physx_PxSweepHit_Pod*' touches;
    ctype='uint32_t' maxNbTouches;
    ctype='uint32_t' nbTouches;
};
struct physx_PxRaycastBuffer_Pod {
    char structgen_pad0[8];
    ctype='physx_PxRaycastHit_Pod' block;
    ctype='bool' hasBlock;
    char structgen_pad1[7];
    ctype='physx_PxRaycastHit_Pod*' touches;
    ctype='uint32_t' maxNbTouches;
    ctype='uint32_t' nbTouches;
};
struct physx_PxOverlapBuffer_Pod {
    char structgen_pad0[8];
    ctype='physx_PxOverlapHit_Pod' block;
    ctype='bool' hasBlock;
    char structgen_pad1[7];
    ctype='physx_PxOverlapHit_Pod*' touches;
    ctype='uint32_t' maxNbTouches;
    ctype='uint32_t' nbTouches;
};
struct physx_PxSweepBuffer_Pod {
    char structgen_pad0[8];
    ctype='physx_PxSweepHit_Pod' block;
    ctype='bool' hasBlock;
    char structgen_pad1[7];
    ctype='physx_PxSweepHit_Pod*' touches;
    ctype='uint32_t' maxNbTouches;
    ctype='uint32_t' nbTouches;
};
struct physx_PxQueryCache_Pod {
    ctype='physx_PxShape_Pod*' shape;
    ctype='physx_PxRigidActor_Pod*' actor;
    ctype='uint32_t' faceIndex;
    char structgen_pad0[4];
};
struct physx_PxQueryFilterData_Pod {
    ctype='physx_PxFilterData_Pod' data;
    ctype='uint16_t' flags;
    char structgen_pad0[2];
};
struct physx_PxQueryFilterCallback_Pod {
    void* vtable_;
};
struct physx_PxRigidDynamic_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxRigidStatic_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxSceneQuerySystem_Pod;
struct physx_PxSceneQueryDesc_Pod {
    ctype='int32_t' staticStructure;
    ctype='int32_t' dynamicStructure;
    ctype='uint32_t' dynamicTreeRebuildRateHint;
    ctype='int32_t' dynamicTreeSecondaryPruner;
    ctype='int32_t' staticBVHBuildStrategy;
    ctype='int32_t' dynamicBVHBuildStrategy;
    ctype='uint32_t' staticNbObjectsPerNode;
    ctype='uint32_t' dynamicNbObjectsPerNode;
    ctype='int32_t' sceneQueryUpdateMode;
};
struct physx_PxSceneQuerySystemBase_Pod {
    void* vtable_;
};
struct physx_PxSceneSQSystem_Pod {
    void* vtable_;
};
struct physx_PxSceneQuerySystem_Pod {
    void* vtable_;
};
struct physx_PxBroadPhaseRegion_Pod {
    ctype='physx_PxBounds3_Pod' mBounds;
    ctype='void*' mUserData;
};
struct physx_PxBroadPhaseRegionInfo_Pod {
    ctype='physx_PxBroadPhaseRegion_Pod' mRegion;
    ctype='uint32_t' mNbStaticObjects;
    ctype='uint32_t' mNbDynamicObjects;
    ctype='bool' mActive;
    ctype='bool' mOverlap;
    char structgen_pad0[6];
};
struct physx_PxBroadPhaseCaps_Pod {
    ctype='uint32_t' mMaxNbRegions;
};
struct physx_PxBroadPhaseDesc_Pod {
    ctype='int32_t' mType;
    char structgen_pad0[4];
    ctype='uint64_t' mContextID;
    char structgen_pad1[8];
    ctype='uint32_t' mFoundLostPairsCapacity;
    ctype='bool' mDiscardStaticVsKinematic;
    ctype='bool' mDiscardKinematicVsKinematic;
    char structgen_pad2[2];
};
struct physx_PxBroadPhaseUpdateData_Pod {
    ctype='uint32_t const*' mCreated;
    ctype='uint32_t' mNbCreated;
    char structgen_pad0[4];
    ctype='uint32_t const*' mUpdated;
    ctype='uint32_t' mNbUpdated;
    char structgen_pad1[4];
    ctype='uint32_t const*' mRemoved;
    ctype='uint32_t' mNbRemoved;
    char structgen_pad2[4];
    ctype='physx_PxBounds3_Pod const*' mBounds;
    ctype='uint32_t const*' mGroups;
    ctype='float const*' mDistances;
    ctype='uint32_t' mCapacity;
    char structgen_pad3[4];
};
struct physx_PxBroadPhasePair_Pod {
    ctype='uint32_t' mID0;
    ctype='uint32_t' mID1;
};
struct physx_PxBroadPhaseResults_Pod {
    ctype='uint32_t' mNbCreatedPairs;
    char structgen_pad0[4];
    ctype='physx_PxBroadPhasePair_Pod const*' mCreatedPairs;
    ctype='uint32_t' mNbDeletedPairs;
    char structgen_pad1[4];
    ctype='physx_PxBroadPhasePair_Pod const*' mDeletedPairs;
};
struct physx_PxBroadPhaseRegions_Pod {
    void* vtable_;
};
struct physx_PxBroadPhase_Pod {
    void* vtable_;
};
struct physx_PxAABBManager_Pod {
    void* vtable_;
};
struct physx_PxBroadPhaseCallback_Pod;
struct physx_PxSimulationEventCallback_Pod;
struct physx_PxSceneLimits_Pod {
    ctype='uint32_t' maxNbActors;
    ctype='uint32_t' maxNbBodies;
    ctype='uint32_t' maxNbStaticShapes;
    ctype='uint32_t' maxNbDynamicShapes;
    ctype='uint32_t' maxNbAggregates;
    ctype='uint32_t' maxNbConstraints;
    ctype='uint32_t' maxNbRegions;
    ctype='uint32_t' maxNbBroadPhaseOverlaps;
};
struct physx_PxgDynamicsMemoryConfig_Pod {
    ctype='uint32_t' tempBufferCapacity;
    ctype='uint32_t' maxRigidContactCount;
    ctype='uint32_t' maxRigidPatchCount;
    ctype='uint32_t' heapCapacity;
    ctype='uint32_t' foundLostPairsCapacity;
    ctype='uint32_t' foundLostAggregatePairsCapacity;
    ctype='uint32_t' totalAggregatePairsCapacity;
    ctype='uint32_t' maxSoftBodyContacts;
    ctype='uint32_t' maxFemClothContacts;
    ctype='uint32_t' maxParticleContacts;
    ctype='uint32_t' collisionStackSize;
    ctype='uint32_t' maxHairContacts;
};
struct physx_PxSceneDesc_Pod {
    ctype='int32_t' staticStructure;
    ctype='int32_t' dynamicStructure;
    ctype='uint32_t' dynamicTreeRebuildRateHint;
    ctype='int32_t' dynamicTreeSecondaryPruner;
    ctype='int32_t' staticBVHBuildStrategy;
    ctype='int32_t' dynamicBVHBuildStrategy;
    ctype='uint32_t' staticNbObjectsPerNode;
    ctype='uint32_t' dynamicNbObjectsPerNode;
    ctype='int32_t' sceneQueryUpdateMode;
    ctype='physx_PxVec3_Pod' gravity;
    ctype='physx_PxSimulationEventCallback_Pod*' simulationEventCallback;
    ctype='physx_PxContactModifyCallback_Pod*' contactModifyCallback;
    ctype='physx_PxCCDContactModifyCallback_Pod*' ccdContactModifyCallback;
    ctype='void const*' filterShaderData;
    ctype='uint32_t' filterShaderDataSize;
    char structgen_pad0[4];
    ctype='void *' filterShader;
    ctype='physx_PxSimulationFilterCallback_Pod*' filterCallback;
    ctype='int32_t' kineKineFilteringMode;
    ctype='int32_t' staticKineFilteringMode;
    ctype='int32_t' broadPhaseType;
    char structgen_pad1[4];
    ctype='physx_PxBroadPhaseCallback_Pod*' broadPhaseCallback;
    ctype='physx_PxSceneLimits_Pod' limits;
    ctype='int32_t' frictionType;
    ctype='int32_t' solverType;
    ctype='float' bounceThresholdVelocity;
    ctype='float' frictionOffsetThreshold;
    ctype='float' frictionCorrelationDistance;
    ctype='uint32_t' flags;
    ctype='physx_PxCpuDispatcher_Pod*' cpuDispatcher;
    char structgen_pad2[8];
    ctype='void*' userData;
    ctype='uint32_t' solverBatchSize;
    ctype='uint32_t' solverArticulationBatchSize;
    ctype='uint32_t' nbContactDataBlocks;
    ctype='uint32_t' maxNbContactDataBlocks;
    ctype='float' maxBiasCoefficient;
    ctype='uint32_t' contactReportStreamBufferSize;
    ctype='uint32_t' ccdMaxPasses;
    ctype='float' ccdThreshold;
    ctype='float' ccdMaxSeparation;
    ctype='float' wakeCounterResetValue;
    ctype='physx_PxBounds3_Pod' sanityBounds;
    ctype='physx_PxgDynamicsMemoryConfig_Pod' gpuDynamicsConfig;
    ctype='uint32_t' gpuMaxNumPartitions;
    ctype='uint32_t' gpuMaxNumStaticPartitions;
    ctype='uint32_t' gpuComputeVersion;
    ctype='uint32_t' contactPairSlabSize;
    ctype='physx_PxSceneQuerySystem_Pod*' sceneQuerySystem;
    char structgen_pad3[8];
};
struct physx_PxSimulationStatistics_Pod {
    ctype='uint32_t' nbActiveConstraints;
    ctype='uint32_t' nbActiveDynamicBodies;
    ctype='uint32_t' nbActiveKinematicBodies;
    ctype='uint32_t' nbStaticBodies;
    ctype='uint32_t' nbDynamicBodies;
    ctype='uint32_t' nbKinematicBodies;
    uint32_t nbShapes[11];
    ctype='uint32_t' nbAggregates;
    ctype='uint32_t' nbArticulations;
    ctype='uint32_t' nbAxisSolverConstraints;
    ctype='uint32_t' compressedContactSize;
    ctype='uint32_t' requiredContactConstraintMemory;
    ctype='uint32_t' peakConstraintMemory;
    ctype='uint32_t' nbDiscreteContactPairsTotal;
    ctype='uint32_t' nbDiscreteContactPairsWithCacheHits;
    ctype='uint32_t' nbDiscreteContactPairsWithContacts;
    ctype='uint32_t' nbNewPairs;
    ctype='uint32_t' nbLostPairs;
    ctype='uint32_t' nbNewTouches;
    ctype='uint32_t' nbLostTouches;
    ctype='uint32_t' nbPartitions;
    char structgen_pad0[4];
    ctype='uint64_t' gpuMemParticles;
    ctype='uint64_t' gpuMemSoftBodies;
    ctype='uint64_t' gpuMemFEMCloths;
    ctype='uint64_t' gpuMemHairSystems;
    ctype='uint64_t' gpuMemHeap;
    ctype='uint64_t' gpuMemHeapBroadPhase;
    ctype='uint64_t' gpuMemHeapNarrowPhase;
    ctype='uint64_t' gpuMemHeapSolver;
    ctype='uint64_t' gpuMemHeapArticulation;
    ctype='uint64_t' gpuMemHeapSimulation;
    ctype='uint64_t' gpuMemHeapSimulationArticulation;
    ctype='uint64_t' gpuMemHeapSimulationParticles;
    ctype='uint64_t' gpuMemHeapSimulationSoftBody;
    ctype='uint64_t' gpuMemHeapSimulationFEMCloth;
    ctype='uint64_t' gpuMemHeapSimulationHairSystem;
    ctype='uint64_t' gpuMemHeapParticles;
    ctype='uint64_t' gpuMemHeapSoftBodies;
    ctype='uint64_t' gpuMemHeapFEMCloths;
    ctype='uint64_t' gpuMemHeapHairSystems;
    ctype='uint64_t' gpuMemHeapOther;
    ctype='uint32_t' nbBroadPhaseAdds;
    ctype='uint32_t' nbBroadPhaseRemoves;
    uint32_t nbDiscreteContactPairs[11][11];
    uint32_t nbCCDPairs[11][11];
    uint32_t nbModifiedContactPairs[11][11];
    uint32_t nbTriggerPairs[11][11];
};
struct physx_PxGpuBodyData_Pod {
    ctype='physx_PxQuat_Pod' quat;
    ctype='physx_PxVec4_Pod' pos;
    ctype='physx_PxVec4_Pod' linVel;
    ctype='physx_PxVec4_Pod' angVel;
};
struct physx_PxGpuActorPair_Pod {
    ctype='uint32_t' srcIndex;
    char structgen_pad0[4];
    ctype='physx_PxNodeIndex_Pod' nodeIndex;
};
struct physx_PxIndexDataPair_Pod {
    ctype='uint32_t' index;
    char structgen_pad0[4];
    ctype='void*' data;
};
struct physx_PxPvdSceneClient_Pod {
    void* vtable_;
};
struct physx_PxContactPairHeader_Pod;
struct physx_PxDominanceGroupPair_Pod {
    ctype='uint8_t' dominance0;
    ctype='uint8_t' dominance1;
};
struct physx_PxBroadPhaseCallback_Pod {
    void* vtable_;
};
struct physx_PxScene_Pod {
    char structgen_pad0[8];
    ctype='void*' userData;
};
struct physx_PxSceneReadLock_Pod {
    char structgen_pad0[8];
};
struct physx_PxSceneWriteLock_Pod {
    char structgen_pad0[8];
};
struct physx_PxContactPairExtraDataItem_Pod {
    ctype='uint8_t' type;
};
struct physx_PxContactPairVelocity_Pod {
    ctype='uint8_t' type;
    char structgen_pad0[3];
    physx_PxVec3_Pod linearVelocity[2];
    physx_PxVec3_Pod angularVelocity[2];
};
struct physx_PxContactPairPose_Pod {
    ctype='uint8_t' type;
    char structgen_pad0[3];
    physx_PxTransform_Pod globalPose[2];
};
struct physx_PxContactPairIndex_Pod {
    ctype='uint8_t' type;
    char structgen_pad0[1];
    ctype='uint16_t' index;
};
struct physx_PxContactPairExtraDataIterator_Pod {
    ctype='uint8_t const*' currPtr;
    ctype='uint8_t const*' endPtr;
    ctype='physx_PxContactPairVelocity_Pod const*' preSolverVelocity;
    ctype='physx_PxContactPairVelocity_Pod const*' postSolverVelocity;
    ctype='physx_PxContactPairPose_Pod const*' eventPose;
    ctype='uint32_t' contactPairIndex;
    char structgen_pad0[4];
};
struct physx_PxContactPair_Pod;
struct physx_PxContactPairHeader_Pod {
    physx_PxActor_Pod* actors[2];
    ctype='uint8_t const*' extraDataStream;
    ctype='uint16_t' extraDataStreamSize;
    ctype='uint16_t' flags;
    char structgen_pad0[4];
    ctype='physx_PxContactPair_Pod const*' pairs;
    ctype='uint32_t' nbPairs;
    char structgen_pad1[4];
};
struct physx_PxContactPairPoint_Pod {
    ctype='physx_PxVec3_Pod' position;
    ctype='float' separation;
    ctype='physx_PxVec3_Pod' normal;
    ctype='uint32_t' internalFaceIndex0;
    ctype='physx_PxVec3_Pod' impulse;
    ctype='uint32_t' internalFaceIndex1;
};
struct physx_PxContactPair_Pod {
    physx_PxShape_Pod* shapes[2];
    ctype='uint8_t const*' contactPatches;
    ctype='uint8_t const*' contactPoints;
    ctype='float const*' contactImpulses;
    ctype='uint32_t' requiredBufferSize;
    ctype='uint8_t' contactCount;
    ctype='uint8_t' patchCount;
    ctype='uint16_t' contactStreamSize;
    ctype='uint16_t' flags;
    ctype='uint16_t' events;
    uint32_t internalData[2];
    char structgen_pad0[4];
};
struct physx_PxTriggerPair_Pod {
    ctype='physx_PxShape_Pod*' triggerShape;
    ctype='physx_PxActor_Pod*' triggerActor;
    ctype='physx_PxShape_Pod*' otherShape;
    ctype='physx_PxActor_Pod*' otherActor;
    ctype='int32_t' status;
    ctype='uint8_t' flags;
    char structgen_pad0[3];
};
struct physx_PxConstraintInfo_Pod {
    ctype='physx_PxConstraint_Pod*' constraint;
    ctype='void*' externalReference;
    ctype='uint32_t' type;
    char structgen_pad0[4];
};
struct physx_PxSimulationEventCallback_Pod {
    void* vtable_;
};
struct physx_PxFEMParameters_Pod {
    ctype='float' velocityDamping;
    ctype='float' settlingThreshold;
    ctype='float' sleepThreshold;
    ctype='float' sleepDamping;
    ctype='float' selfCollisionFilterDistance;
    ctype='float' selfCollisionStressTolerance;
};
struct physx_PxPruningStructure_Pod {
    char structgen_pad0[16];
};
struct physx_PxExtendedVec3_Pod {
    ctype='double' x;
    ctype='double' y;
    ctype='double' z;
};
struct physx_PxControllerManager_Pod;
struct physx_PxObstacle_Pod {
    char structgen_pad0[8];
    ctype='void*' mUserData;
    ctype='physx_PxExtendedVec3_Pod' mPos;
    ctype='physx_PxQuat_Pod' mRot;
};
struct physx_PxBoxObstacle_Pod {
    char structgen_pad0[8];
    ctype='void*' mUserData;
    ctype='physx_PxExtendedVec3_Pod' mPos;
    ctype='physx_PxQuat_Pod' mRot;
    ctype='physx_PxVec3_Pod' mHalfExtents;
    char structgen_pad1[4];
};
struct physx_PxCapsuleObstacle_Pod {
    char structgen_pad0[8];
    ctype='void*' mUserData;
    ctype='physx_PxExtendedVec3_Pod' mPos;
    ctype='physx_PxQuat_Pod' mRot;
    ctype='float' mHalfHeight;
    ctype='float' mRadius;
};
struct physx_PxObstacleContext_Pod {
    void* vtable_;
};
struct physx_PxController_Pod;
struct physx_PxControllerBehaviorCallback_Pod;
struct physx_PxControllerState_Pod {
    ctype='physx_PxVec3_Pod' deltaXP;
    char structgen_pad0[4];
    ctype='physx_PxShape_Pod*' touchedShape;
    ctype='physx_PxRigidActor_Pod*' touchedActor;
    ctype='uint32_t' touchedObstacleHandle;
    ctype='uint32_t' collisionFlags;
    ctype='bool' standOnAnotherCCT;
    ctype='bool' standOnObstacle;
    ctype='bool' isMovingUp;
    char structgen_pad1[5];
};
struct physx_PxControllerStats_Pod {
    ctype='uint16_t' nbIterations;
    ctype='uint16_t' nbFullUpdates;
    ctype='uint16_t' nbPartialUpdates;
    ctype='uint16_t' nbTessellation;
};
struct physx_PxControllerHit_Pod {
    ctype='physx_PxController_Pod*' controller;
    ctype='physx_PxExtendedVec3_Pod' worldPos;
    ctype='physx_PxVec3_Pod' worldNormal;
    ctype='physx_PxVec3_Pod' dir;
    ctype='float' length;
    char structgen_pad0[4];
};
struct physx_PxControllerShapeHit_Pod {
    ctype='physx_PxController_Pod*' controller;
    ctype='physx_PxExtendedVec3_Pod' worldPos;
    ctype='physx_PxVec3_Pod' worldNormal;
    ctype='physx_PxVec3_Pod' dir;
    ctype='float' length;
    char structgen_pad0[4];
    ctype='physx_PxShape_Pod*' shape;
    ctype='physx_PxRigidActor_Pod*' actor;
    ctype='uint32_t' triangleIndex;
    char structgen_pad1[4];
};
struct physx_PxControllersHit_Pod {
    ctype='physx_PxController_Pod*' controller;
    ctype='physx_PxExtendedVec3_Pod' worldPos;
    ctype='physx_PxVec3_Pod' worldNormal;
    ctype='physx_PxVec3_Pod' dir;
    ctype='float' length;
    char structgen_pad0[4];
    ctype='physx_PxController_Pod*' other;
};
struct physx_PxControllerObstacleHit_Pod {
    ctype='physx_PxController_Pod*' controller;
    ctype='physx_PxExtendedVec3_Pod' worldPos;
    ctype='physx_PxVec3_Pod' worldNormal;
    ctype='physx_PxVec3_Pod' dir;
    ctype='float' length;
    char structgen_pad0[4];
    ctype='void const*' userData;
};
struct physx_PxUserControllerHitReport_Pod {
    void* vtable_;
};
struct physx_PxControllerFilterCallback_Pod {
    void* vtable_;
};
struct physx_PxControllerFilters_Pod {
    ctype='physx_PxFilterData_Pod const*' mFilterData;
    ctype='physx_PxQueryFilterCallback_Pod*' mFilterCallback;
    ctype='uint16_t' mFilterFlags;
    char structgen_pad0[6];
    ctype='physx_PxControllerFilterCallback_Pod*' mCCTFilterCallback;
};
struct physx_PxControllerDesc_Pod {
    char structgen_pad0[8];
    ctype='physx_PxExtendedVec3_Pod' position;
    ctype='physx_PxVec3_Pod' upDirection;
    ctype='float' slopeLimit;
    ctype='float' invisibleWallHeight;
    ctype='float' maxJumpHeight;
    ctype='float' contactOffset;
    ctype='float' stepOffset;
    ctype='float' density;
    ctype='float' scaleCoeff;
    ctype='float' volumeGrowth;
    char structgen_pad1[4];
    ctype='physx_PxUserControllerHitReport_Pod*' reportCallback;
    ctype='physx_PxControllerBehaviorCallback_Pod*' behaviorCallback;
    ctype='int32_t' nonWalkableMode;
    char structgen_pad2[4];
    ctype='physx_PxMaterial_Pod*' material;
    ctype='bool' registerDeletionListener;
    ctype='uint8_t' clientID;
    char structgen_pad3[6];
    ctype='void*' userData;
    char structgen_pad4[8];
};
struct physx_PxController_Pod {
    void* vtable_;
};
struct physx_PxBoxControllerDesc_Pod {
    char structgen_pad0[8];
    ctype='physx_PxExtendedVec3_Pod' position;
    ctype='physx_PxVec3_Pod' upDirection;
    ctype='float' slopeLimit;
    ctype='float' invisibleWallHeight;
    ctype='float' maxJumpHeight;
    ctype='float' contactOffset;
    ctype='float' stepOffset;
    ctype='float' density;
    ctype='float' scaleCoeff;
    ctype='float' volumeGrowth;
    char structgen_pad1[4];
    ctype='physx_PxUserControllerHitReport_Pod*' reportCallback;
    ctype='physx_PxControllerBehaviorCallback_Pod*' behaviorCallback;
    ctype='int32_t' nonWalkableMode;
    char structgen_pad2[4];
    ctype='physx_PxMaterial_Pod*' material;
    ctype='bool' registerDeletionListener;
    ctype='uint8_t' clientID;
    char structgen_pad3[6];
    ctype='void*' userData;
    char structgen_pad4[4];
    ctype='float' halfHeight;
    ctype='float' halfSideExtent;
    ctype='float' halfForwardExtent;
};
struct physx_PxBoxController_Pod {
    void* vtable_;
};
struct physx_PxCapsuleControllerDesc_Pod {
    char structgen_pad0[8];
    ctype='physx_PxExtendedVec3_Pod' position;
    ctype='physx_PxVec3_Pod' upDirection;
    ctype='float' slopeLimit;
    ctype='float' invisibleWallHeight;
    ctype='float' maxJumpHeight;
    ctype='float' contactOffset;
    ctype='float' stepOffset;
    ctype='float' density;
    ctype='float' scaleCoeff;
    ctype='float' volumeGrowth;
    char structgen_pad1[4];
    ctype='physx_PxUserControllerHitReport_Pod*' reportCallback;
    ctype='physx_PxControllerBehaviorCallback_Pod*' behaviorCallback;
    ctype='int32_t' nonWalkableMode;
    char structgen_pad2[4];
    ctype='physx_PxMaterial_Pod*' material;
    ctype='bool' registerDeletionListener;
    ctype='uint8_t' clientID;
    char structgen_pad3[6];
    ctype='void*' userData;
    char structgen_pad4[4];
    ctype='float' radius;
    ctype='float' height;
    ctype='int32_t' climbingMode;
};
struct physx_PxCapsuleController_Pod {
    void* vtable_;
};
struct physx_PxControllerBehaviorCallback_Pod {
    void* vtable_;
};
struct physx_PxControllerManager_Pod {
    void* vtable_;
};
struct physx_PxDim3_Pod {
    ctype='uint32_t' x;
    ctype='uint32_t' y;
    ctype='uint32_t' z;
};
struct physx_PxSDFDesc_Pod {
    ctype='physx_PxBoundedData_Pod' sdf;
    ctype='physx_PxDim3_Pod' dims;
    ctype='physx_PxVec3_Pod' meshLower;
    ctype='float' spacing;
    ctype='uint32_t' subgridSize;
    ctype='int32_t' bitsPerSubgridPixel;
    ctype='physx_PxDim3_Pod' sdfSubgrids3DTexBlockDim;
    ctype='physx_PxBoundedData_Pod' sdfSubgrids;
    ctype='physx_PxBoundedData_Pod' sdfStartSlots;
    ctype='float' subgridsMinSdfValue;
    ctype='float' subgridsMaxSdfValue;
    ctype='physx_PxBounds3_Pod' sdfBounds;
    ctype='float' narrowBandThicknessRelativeToSdfBoundsDiagonal;
    ctype='uint32_t' numThreadsForSdfConstruction;
};
struct physx_PxConvexMeshDesc_Pod {
    ctype='physx_PxBoundedData_Pod' points;
    ctype='physx_PxBoundedData_Pod' polygons;
    ctype='physx_PxBoundedData_Pod' indices;
    ctype='uint16_t' flags;
    ctype='uint16_t' vertexLimit;
    ctype='uint16_t' polygonLimit;
    ctype='uint16_t' quantizedCount;
    ctype='physx_PxSDFDesc_Pod*' sdfDesc;
};
struct physx_PxTriangleMeshDesc_Pod {
    ctype='physx_PxBoundedData_Pod' points;
    ctype='physx_PxBoundedData_Pod' triangles;
    ctype='uint16_t' flags;
    char structgen_pad0[22];
    ctype='physx_PxSDFDesc_Pod*' sdfDesc;
};
struct physx_PxTetrahedronMeshDesc_Pod {
    char structgen_pad0[16];
    ctype='physx_PxBoundedData_Pod' points;
    ctype='physx_PxBoundedData_Pod' tetrahedrons;
    ctype='uint16_t' flags;
    ctype='uint16_t' tetsPerElement;
    char structgen_pad1[4];
};
struct physx_PxSoftBodySimulationDataDesc_Pod {
    ctype='physx_PxBoundedData_Pod' vertexToTet;
};
struct physx_PxBVH34MidphaseDesc_Pod {
    ctype='uint32_t' numPrimsPerLeaf;
    ctype='int32_t' buildStrategy;
    ctype='bool' quantized;
    char structgen_pad0[3];
};
struct physx_PxMidphaseDesc_Pod {
    char structgen_pad0[16];
};
struct physx_PxBVHDesc_Pod {
    ctype='physx_PxBoundedData_Pod' bounds;
    ctype='float' enlargement;
    ctype='uint32_t' numPrimsPerLeaf;
    ctype='int32_t' buildStrategy;
    char structgen_pad0[4];
};
struct physx_PxCookingParams_Pod {
    ctype='float' areaTestEpsilon;
    ctype='float' planeTolerance;
    ctype='int32_t' convexMeshCookingType;
    ctype='bool' suppressTriangleMeshRemapTable;
    ctype='bool' buildTriangleAdjacencies;
    ctype='bool' buildGPUData;
    char structgen_pad0[1];
    ctype='physx_PxTolerancesScale_Pod' scale;
    ctype='uint32_t' meshPreprocessParams;
    ctype='float' meshWeldTolerance;
    ctype='physx_PxMidphaseDesc_Pod' midphaseDesc;
    ctype='uint32_t' gaussMapLimit;
    ctype='float' maxWeightRatioInTet;
};
struct physx_PxDefaultMemoryOutputStream_Pod {
    char structgen_pad0[32];
};
struct physx_PxDefaultMemoryInputData_Pod {
    char structgen_pad0[32];
};
struct physx_PxDefaultFileOutputStream_Pod {
    char structgen_pad0[16];
};
struct physx_PxDefaultFileInputData_Pod {
    char structgen_pad0[24];
};
struct physx_PxDefaultAllocator_Pod {
    void* vtable_;
};
struct physx_PxJoint_Pod;
struct physx_PxRackAndPinionJoint_Pod;
struct physx_PxGearJoint_Pod;
struct physx_PxD6Joint_Pod;
struct physx_PxDistanceJoint_Pod;
struct physx_PxContactJoint_Pod;
struct physx_PxFixedJoint_Pod;
struct physx_PxPrismaticJoint_Pod;
struct physx_PxRevoluteJoint_Pod;
struct physx_PxSphericalJoint_Pod;
struct physx_PxJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxSpring_Pod {
    ctype='float' stiffness;
    ctype='float' damping;
};
struct physx_PxDistanceJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxJacobianRow_Pod {
    ctype='physx_PxVec3_Pod' linear0;
    ctype='physx_PxVec3_Pod' linear1;
    ctype='physx_PxVec3_Pod' angular0;
    ctype='physx_PxVec3_Pod' angular1;
};
struct physx_PxContactJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxFixedJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxJointLimitParameters_Pod {
    ctype='float' restitution;
    ctype='float' bounceThreshold;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' contactDistance_deprecated;
};
struct physx_PxJointLinearLimit_Pod {
    ctype='float' restitution;
    ctype='float' bounceThreshold;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' contactDistance_deprecated;
    ctype='float' value;
};
struct physx_PxJointLinearLimitPair_Pod {
    ctype='float' restitution;
    ctype='float' bounceThreshold;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' contactDistance_deprecated;
    ctype='float' upper;
    ctype='float' lower;
};
struct physx_PxJointAngularLimitPair_Pod {
    ctype='float' restitution;
    ctype='float' bounceThreshold;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' contactDistance_deprecated;
    ctype='float' upper;
    ctype='float' lower;
};
struct physx_PxJointLimitCone_Pod {
    ctype='float' restitution;
    ctype='float' bounceThreshold;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' contactDistance_deprecated;
    ctype='float' yAngle;
    ctype='float' zAngle;
};
struct physx_PxJointLimitPyramid_Pod {
    ctype='float' restitution;
    ctype='float' bounceThreshold;
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' contactDistance_deprecated;
    ctype='float' yAngleMin;
    ctype='float' yAngleMax;
    ctype='float' zAngleMin;
    ctype='float' zAngleMax;
};
struct physx_PxPrismaticJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxRevoluteJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxSphericalJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxD6JointDrive_Pod {
    ctype='float' stiffness;
    ctype='float' damping;
    ctype='float' forceLimit;
    ctype='uint32_t' flags;
};
struct physx_PxD6Joint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxGearJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxRackAndPinionJoint_Pod {
    char structgen_pad0[16];
    ctype='void*' userData;
};
struct physx_PxGroupsMask_Pod {
    ctype='uint16_t' bits0;
    ctype='uint16_t' bits1;
    ctype='uint16_t' bits2;
    ctype='uint16_t' bits3;
};
struct physx_PxDefaultErrorCallback_Pod {
    void* vtable_;
};
struct physx_PxRigidActorExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxMassProperties_Pod {
    ctype='physx_PxMat33_Pod' inertiaTensor;
    ctype='physx_PxVec3_Pod' centerOfMass;
    ctype='float' mass;
};
struct physx_PxRigidBodyExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxShapeExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxMeshOverlapUtil_Pod {
    char structgen_pad0[1040];
};
struct physx_PxBinaryConverter_Pod;
struct physx_PxXmlMiscParameter_Pod {
    ctype='physx_PxVec3_Pod' upVector;
    ctype='physx_PxTolerancesScale_Pod' scale;
};
struct physx_PxSerialization_Pod {
    char structgen_pad0[1];
};
struct physx_PxDefaultCpuDispatcher_Pod {
    void* vtable_;
};
struct physx_PxStringTableExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxBroadPhaseExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxSceneQueryExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxBatchQueryExt_Pod {
    void* vtable_;
};
struct physx_PxCustomSceneQuerySystem_Pod {
    void* vtable_;
};
struct physx_PxCustomSceneQuerySystemAdapter_Pod {
    void* vtable_;
};
struct physx_PxSamplingExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxPoissonSampler_Pod {
    char structgen_pad0[8];
};
struct physx_PxTriangleMeshPoissonSampler_Pod {
    char structgen_pad0[8];
};
struct physx_PxTetrahedronMeshExt_Pod {
    char structgen_pad0[1];
};
struct physx_PxRepXObject_Pod {
    ctype='char const*' typeName;
    ctype='void const*' serializable;
    ctype='uint64_t' id;
};
struct physx_PxCooking_Pod;
struct physx_PxRepXInstantiationArgs_Pod {
    char structgen_pad0[8];
    ctype='physx_PxCooking_Pod*' cooker;
    ctype='physx_PxStringTable_Pod*' stringTable;
};
struct physx_XmlMemoryAllocator_Pod;
struct physx_XmlWriter_Pod;
struct physx_XmlReader_Pod;
struct physx_MemoryBuffer_Pod;
struct physx_PxRepXSerializer_Pod {
    void* vtable_;
};
struct physx_PxVehicleWheels4SimData_Pod;
struct physx_PxVehicleWheels4DynData_Pod;
struct physx_PxVehicleTireForceCalculator_Pod;
struct physx_PxVehicleDrivableSurfaceToTireFrictionPairs_Pod;
struct physx_PxVehicleTelemetryData_Pod;
struct physx_PxPvdTransport_Pod;
struct physx_PxPvd_Pod {
    void* vtable_;
};
struct physx_PxPvdTransport_Pod {
    void* vtable_;
};
