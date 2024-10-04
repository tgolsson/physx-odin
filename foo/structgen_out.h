#include <stdint.h>
#include "physx_generated_enums.h"
#define FORWARD_DECL_STRUCT(type) typedef struct physx_##type physx_##type;
FORWARD_DECL_STRUCT(PxProfilerCallback);
FORWARD_DECL_STRUCT(PxTriangleMesh);
FORWARD_DECL_STRUCT(PxHeightField);
FORWARD_DECL_STRUCT(PxBroadPhaseCallback);
FORWARD_DECL_STRUCT(PxContactModifyCallback);
FORWARD_DECL_STRUCT(PxCpuDispatcher);
FORWARD_DECL_STRUCT(PxCCDContactModifyCallback);
FORWARD_DECL_STRUCT(PxSimulationEventCallback);
FORWARD_DECL_STRUCT(PxTetrahedronMesh);
FORWARD_DECL_STRUCT(PxCustomGeometryCallbacks);
FORWARD_DECL_STRUCT(PxSimulationFilterCallback);
FORWARD_DECL_STRUCT(PxSceneQuerySystem);
FORWARD_DECL_STRUCT(PxContactPair);
FORWARD_DECL_STRUCT(PxController);
FORWARD_DECL_STRUCT(PxQueryFilterCallback);
FORWARD_DECL_STRUCT(PxControllerFilterCallback);
FORWARD_DECL_STRUCT(PxControllerBehaviorCallback);
FORWARD_DECL_STRUCT(PxUserControllerHitReport);
FORWARD_DECL_STRUCT(PxCooking);
FORWARD_DECL_STRUCT(PxStringTable);
typedef struct physx_PxAllocator {
    char structgen_pad0[1];
} physx_PxAllocator;
typedef struct physx_PxRawAllocator {
    char structgen_pad0[1];
} physx_PxRawAllocator;
typedef struct physx_PxVirtualAllocator {
    char structgen_pad0[16];
} physx_PxVirtualAllocator;
typedef struct physx_PxUserAllocated {
    char structgen_pad0[1];
} physx_PxUserAllocated;
typedef struct physx_PxTempAllocator {
    char structgen_pad0[1];
} physx_PxTempAllocator;
typedef struct physx_PxBitAndByte {
    char structgen_pad0[1];
} physx_PxBitAndByte;
typedef struct physx_PxBitMap {
    char structgen_pad0[16];
} physx_PxBitMap;
typedef struct physx_PxVec3 {
    float x;
    float y;
    float z;
} physx_PxVec3;
typedef struct physx_PxVec3Padded {
    float x;
    float y;
    float z;
    uint32_t padding;
} physx_PxVec3Padded;
typedef struct physx_PxQuat {
    float x;
    float y;
    float z;
    float w;
} physx_PxQuat;
typedef struct physx_PxTransform {
    physx_PxQuat q;
    physx_PxVec3 p;
} physx_PxTransform;
typedef struct physx_PxTransformPadded {
    physx_PxTransform transform;
    uint32_t padding;
} physx_PxTransformPadded;
typedef struct physx_PxMat33 {
    physx_PxVec3 column0;
    physx_PxVec3 column1;
    physx_PxVec3 column2;
} physx_PxMat33;
typedef struct physx_PxBounds3 {
    physx_PxVec3 minimum;
    physx_PxVec3 maximum;
} physx_PxBounds3;
typedef struct physx_PxBroadcastingAllocator {
    char structgen_pad0[176];
} physx_PxBroadcastingAllocator;
typedef struct physx_PxBroadcastingErrorCallback {
    char structgen_pad0[160];
} physx_PxBroadcastingErrorCallback;
typedef struct physx_PxVec4 {
    float x;
    float y;
    float z;
    float w;
} physx_PxVec4;
typedef struct physx_PxMat44 {
    physx_PxVec4 column0;
    physx_PxVec4 column1;
    physx_PxVec4 column2;
    physx_PxVec4 column3;
} physx_PxMat44;
typedef struct physx_PxPlane {
    physx_PxVec3 n;
    float d;
} physx_PxPlane;
typedef struct physx_Interpolation {
    char structgen_pad0[1];
} physx_Interpolation;
typedef struct physx_PxMutexImpl {
    char structgen_pad0[1];
} physx_PxMutexImpl;
typedef struct physx_PxReadWriteLock {
    char structgen_pad0[8];
} physx_PxReadWriteLock;
typedef struct physx_PxProfileScoped {
    physx_PxProfilerCallback* mCallback;
    char const* mEventName;
    void* mProfilerData;
    uint64_t mContextId;
    bool mDetached;
    char structgen_pad0[7];
} physx_PxProfileScoped;
typedef struct physx_PxSListEntry {
    char structgen_pad0[16];
} physx_PxSListEntry;
typedef struct physx_PxSListImpl {
    char structgen_pad0[1];
} physx_PxSListImpl;
typedef struct physx_PxSyncImpl {
    char structgen_pad0[1];
} physx_PxSyncImpl;
typedef struct physx_PxCounterFrequencyToTensOfNanos {
    uint64_t mNumerator;
    uint64_t mDenominator;
} physx_PxCounterFrequencyToTensOfNanos;
typedef struct physx_PxTime {
    char structgen_pad0[8];
} physx_PxTime;
typedef struct physx_PxVec2 {
    float x;
    float y;
} physx_PxVec2;
typedef struct physx_PxStridedData {
    uint32_t stride;
    char structgen_pad0[4];
    void const* data;
} physx_PxStridedData;
typedef struct physx_PxBoundedData {
    uint32_t stride;
    char structgen_pad0[4];
    void const* data;
    uint32_t count;
    char structgen_pad1[4];
} physx_PxBoundedData;
typedef struct physx_PxDebugPoint {
    physx_PxVec3 pos;
    uint32_t color;
} physx_PxDebugPoint;
typedef struct physx_PxDebugLine {
    physx_PxVec3 pos0;
    uint32_t color0;
    physx_PxVec3 pos1;
    uint32_t color1;
} physx_PxDebugLine;
typedef struct physx_PxDebugTriangle {
    physx_PxVec3 pos0;
    uint32_t color0;
    physx_PxVec3 pos1;
    uint32_t color1;
    physx_PxVec3 pos2;
    uint32_t color2;
} physx_PxDebugTriangle;
typedef struct physx_PxDebugText {
    physx_PxVec3 position;
    float size;
    uint32_t color;
    char structgen_pad0[4];
    char const* string;
} physx_PxDebugText;
typedef struct physx_PxDeserializationContext {
    char structgen_pad0[16];
} physx_PxDeserializationContext;
typedef struct physx_PxBase {
    char structgen_pad0[16];
} physx_PxBase;
typedef struct physx_PxRefCounted {
    char structgen_pad0[16];
} physx_PxRefCounted;
typedef struct physx_PxTolerancesScale {
    float length;
    float speed;
} physx_PxTolerancesScale;
typedef struct physx_PxMetaDataEntry {
    char const* type_;
    char const* name;
    uint32_t offset;
    uint32_t size;
    uint32_t count;
    uint32_t offsetSize;
    uint32_t flags;
    uint32_t alignment;
} physx_PxMetaDataEntry;
typedef struct physx_PxBaseTask {
    char structgen_pad0[24];
} physx_PxBaseTask;
typedef struct physx_PxTask {
    char structgen_pad0[32];
} physx_PxTask;
typedef struct physx_PxLightCpuTask {
    char structgen_pad0[40];
} physx_PxLightCpuTask;
typedef struct physx_PxGeometry {
    char structgen_pad0[4];
    float mTypePadding;
} physx_PxGeometry;
typedef struct physx_PxBoxGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxVec3 halfExtents;
} physx_PxBoxGeometry;
typedef struct physx_PxBVH {
    char structgen_pad0[16];
} physx_PxBVH;
typedef struct physx_PxCapsuleGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    float radius;
    float halfHeight;
} physx_PxCapsuleGeometry;
typedef struct physx_PxHullPolygon {
    float mPlane[4];
    uint16_t mNbVerts;
    uint16_t mIndexBase;
} physx_PxHullPolygon;
typedef struct physx_PxConvexMesh {
    char structgen_pad0[16];
} physx_PxConvexMesh;
typedef struct physx_PxMeshScale {
    physx_PxVec3 scale;
    physx_PxQuat rotation;
} physx_PxMeshScale;
typedef struct physx_PxConvexMeshGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxMeshScale scale;
    char structgen_pad1[4];
    physx_PxConvexMesh* convexMesh;
    uint8_t meshFlags;
    char structgen_pad2[7];
} physx_PxConvexMeshGeometry;
typedef struct physx_PxSphereGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    float radius;
} physx_PxSphereGeometry;
typedef struct physx_PxPlaneGeometry {
    char structgen_pad0[4];
    float mTypePadding;
} physx_PxPlaneGeometry;
typedef struct physx_PxTriangleMeshGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxMeshScale scale;
    uint8_t meshFlags;
    char structgen_pad1[3];
    physx_PxTriangleMesh* triangleMesh;
} physx_PxTriangleMeshGeometry;
typedef struct physx_PxHeightFieldGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxHeightField* heightField;
    float heightScale;
    float rowScale;
    float columnScale;
    uint8_t heightFieldFlags;
    char structgen_pad1[3];
} physx_PxHeightFieldGeometry;
typedef struct physx_PxParticleSystemGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    int32_t mSolverType;
} physx_PxParticleSystemGeometry;
typedef struct physx_PxHairSystemGeometry {
    char structgen_pad0[4];
    float mTypePadding;
} physx_PxHairSystemGeometry;
typedef struct physx_PxTetrahedronMeshGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxTetrahedronMesh* tetrahedronMesh;
} physx_PxTetrahedronMeshGeometry;
typedef struct physx_PxQueryHit {
    uint32_t faceIndex;
} physx_PxQueryHit;
typedef struct physx_PxLocationHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
} physx_PxLocationHit;
typedef struct physx_PxGeomRaycastHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
    float u;
    float v;
} physx_PxGeomRaycastHit;
typedef struct physx_PxGeomOverlapHit {
    uint32_t faceIndex;
} physx_PxGeomOverlapHit;
typedef struct physx_PxGeomSweepHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
} physx_PxGeomSweepHit;
typedef struct physx_PxGeomIndexPair {
    uint32_t id0;
    uint32_t id1;
} physx_PxGeomIndexPair;
typedef struct physx_PxQueryThreadContext {
    char structgen_pad0[1];
} physx_PxQueryThreadContext;
typedef struct physx_PxCustomGeometryType {
    char structgen_pad0[4];
} physx_PxCustomGeometryType;
typedef struct physx_PxCustomGeometry {
    char structgen_pad0[4];
    float mTypePadding;
    physx_PxCustomGeometryCallbacks* callbacks;
} physx_PxCustomGeometry;
typedef struct physx_PxGeometryHolder {
    char structgen_pad0[56];
} physx_PxGeometryHolder;
typedef struct physx_PxGeometryQuery {
    char structgen_pad0[1];
} physx_PxGeometryQuery;
typedef struct physx_PxHeightFieldSample {
    int16_t height;
    physx_PxBitAndByte materialIndex0;
    physx_PxBitAndByte materialIndex1;
} physx_PxHeightFieldSample;
typedef struct physx_PxHeightField {
    char structgen_pad0[16];
} physx_PxHeightField;
typedef struct physx_PxHeightFieldDesc {
    uint32_t nbRows;
    uint32_t nbColumns;
    int32_t format;
    char structgen_pad0[4];
    physx_PxStridedData samples;
    float convexEdgeThreshold;
    uint16_t flags;
    char structgen_pad1[2];
} physx_PxHeightFieldDesc;
typedef struct physx_PxMeshQuery {
    char structgen_pad0[1];
} physx_PxMeshQuery;
typedef struct physx_PxSimpleTriangleMesh {
    physx_PxBoundedData points;
    physx_PxBoundedData triangles;
    uint16_t flags;
    char structgen_pad0[6];
} physx_PxSimpleTriangleMesh;
typedef struct physx_PxTriangle {
    physx_PxVec3 verts[3];
} physx_PxTriangle;
typedef struct physx_PxTrianglePadded {
    physx_PxVec3 verts[3];
    uint32_t padding;
} physx_PxTrianglePadded;
typedef struct physx_PxTriangleMesh {
    char structgen_pad0[16];
} physx_PxTriangleMesh;
typedef struct physx_PxBVH34TriangleMesh {
    char structgen_pad0[16];
} physx_PxBVH34TriangleMesh;
typedef struct physx_PxTetrahedron {
    physx_PxVec3 verts[4];
} physx_PxTetrahedron;
typedef struct physx_PxSoftBodyAuxData {
    char structgen_pad0[16];
} physx_PxSoftBodyAuxData;
typedef struct physx_PxTetrahedronMesh {
    char structgen_pad0[16];
} physx_PxTetrahedronMesh;
typedef struct physx_PxSoftBodyMesh {
    char structgen_pad0[16];
} physx_PxSoftBodyMesh;
typedef struct physx_PxCollisionMeshMappingData {
    char structgen_pad0[8];
} physx_PxCollisionMeshMappingData;
typedef struct physx_PxSoftBodyCollisionData {
    char structgen_pad0[1];
} physx_PxSoftBodyCollisionData;
typedef struct physx_PxTetrahedronMeshData {
    char structgen_pad0[1];
} physx_PxTetrahedronMeshData;
typedef struct physx_PxSoftBodySimulationData {
    char structgen_pad0[1];
} physx_PxSoftBodySimulationData;
typedef struct physx_PxCollisionTetrahedronMeshData {
    char structgen_pad0[8];
} physx_PxCollisionTetrahedronMeshData;
typedef struct physx_PxSimulationTetrahedronMeshData {
    char structgen_pad0[8];
} physx_PxSimulationTetrahedronMeshData;
typedef struct physx_PxActor {
    char structgen_pad0[16];
    void* userData;
} physx_PxActor;
typedef struct physx_PxAggregate {
    char structgen_pad0[16];
    void* userData;
} physx_PxAggregate;
typedef struct physx_PxSpringModifiers {
    float stiffness;
    float damping;
    char structgen_pad0[8];
} physx_PxSpringModifiers;
typedef struct physx_PxRestitutionModifiers {
    float restitution;
    float velocityThreshold;
    char structgen_pad0[8];
} physx_PxRestitutionModifiers;
typedef struct physx_Px1DConstraint {
    physx_PxVec3 linear0;
    float geometricError;
    physx_PxVec3 angular0;
    float velocityTarget;
    physx_PxVec3 linear1;
    float minImpulse;
    physx_PxVec3 angular1;
    float maxImpulse;
    physx_Px1DConstraintMods mods;
    float forInternalUse;
    uint16_t flags;
    uint16_t solveHint;
    char structgen_pad0[8];
} physx_Px1DConstraint;
typedef struct physx_PxConstraintInvMassScale {
    float linear0;
    float angular0;
    float linear1;
    float angular1;
} physx_PxConstraintInvMassScale;
typedef struct physx_PxContactPoint {
    physx_PxVec3 normal;
    float separation;
    physx_PxVec3 point;
    float maxImpulse;
    physx_PxVec3 targetVel;
    float staticFriction;
    uint8_t materialFlags;
    char structgen_pad0[3];
    uint32_t internalFaceIndex1;
    float dynamicFriction;
    float restitution;
    float damping;
    char structgen_pad1[12];
} physx_PxContactPoint;
typedef struct physx_PxSolverBody {
    physx_PxVec3 linearVelocity;
    uint16_t maxSolverNormalProgress;
    uint16_t maxSolverFrictionProgress;
    physx_PxVec3 angularState;
    uint32_t solverProgress;
} physx_PxSolverBody;
typedef struct physx_PxSolverBodyData {
    physx_PxVec3 linearVelocity;
    float invMass;
    physx_PxVec3 angularVelocity;
    float reportThreshold;
    physx_PxMat33 sqrtInvInertia;
    float penBiasClamp;
    uint32_t nodeIndex;
    float maxContactImpulse;
    physx_PxTransform body2World;
    uint16_t pad;
    char structgen_pad0[2];
} physx_PxSolverBodyData;
typedef struct physx_PxConstraintBatchHeader {
    uint32_t startIndex;
    uint16_t stride;
    uint16_t constraintType;
} physx_PxConstraintBatchHeader;
typedef struct physx_PxSolverConstraintDesc {
    char structgen_pad0[16];
    uint32_t bodyADataIndex;
    uint32_t bodyBDataIndex;
    uint32_t linkIndexA;
    uint32_t linkIndexB;
    uint8_t* constraint;
    void* writeBack;
    uint16_t progressA;
    uint16_t progressB;
    uint16_t constraintLengthOver16;
    uint8_t padding[10];
} physx_PxSolverConstraintDesc;
typedef struct physx_PxSolverConstraintPrepDescBase {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxSolverBody const* body0;
    physx_PxSolverBody const* body1;
    physx_PxSolverBodyData const* data0;
    physx_PxSolverBodyData const* data1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    char structgen_pad0[8];
} physx_PxSolverConstraintPrepDescBase;
typedef struct physx_PxSolverConstraintPrepDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxSolverBody const* body0;
    physx_PxSolverBody const* body1;
    physx_PxSolverBodyData const* data0;
    physx_PxSolverBodyData const* data1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    char structgen_pad0[8];
    physx_Px1DConstraint* rows;
    uint32_t numRows;
    float linBreakForce;
    float angBreakForce;
    float minResponseThreshold;
    void* writeback;
    bool disablePreprocessing;
    bool improvedSlerp;
    bool driveLimitsAreForces;
    bool extendedLimits;
    bool disableConstraint;
    char structgen_pad1[3];
    physx_PxVec3Padded body0WorldOffset;
    char structgen_pad2[8];
} physx_PxSolverConstraintPrepDesc;
typedef struct physx_PxSolverContactDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxSolverBody const* body0;
    physx_PxSolverBody const* body1;
    physx_PxSolverBodyData const* data0;
    physx_PxSolverBodyData const* data1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    void* shapeInteraction;
    physx_PxContactPoint* contacts;
    uint32_t numContacts;
    bool hasMaxImpulse;
    bool disableStrongFriction;
    bool hasForceThresholds;
    char structgen_pad0[1];
    float restDistance;
    float maxCCDSeparation;
    uint8_t* frictionPtr;
    uint8_t frictionCount;
    char structgen_pad1[7];
    float* contactForces;
    uint32_t startFrictionPatchIndex;
    uint32_t numFrictionPatches;
    uint32_t startContactPatchIndex;
    uint16_t numContactPatches;
    uint16_t axisConstraintCount;
    float offsetSlop;
    char structgen_pad2[12];
} physx_PxSolverContactDesc;
typedef struct physx_PxArticulationLimit {
    float low;
    float high;
} physx_PxArticulationLimit;
typedef struct physx_PxArticulationDrive {
    float stiffness;
    float damping;
    float maxForce;
    int32_t driveType;
} physx_PxArticulationDrive;
typedef struct physx_PxTGSSolverBodyVel {
    physx_PxVec3 linearVelocity;
    uint16_t nbStaticInteractions;
    uint16_t maxDynamicPartition;
    physx_PxVec3 angularVelocity;
    uint32_t partitionMask;
    physx_PxVec3 deltaAngDt;
    float maxAngVel;
    physx_PxVec3 deltaLinDt;
    uint16_t lockFlags;
    bool isKinematic;
    uint8_t pad;
} physx_PxTGSSolverBodyVel;
typedef struct physx_PxTGSSolverBodyTxInertia {
    physx_PxTransform deltaBody2World;
    physx_PxMat33 sqrtInvInertia;
} physx_PxTGSSolverBodyTxInertia;
typedef struct physx_PxTGSSolverBodyData {
    physx_PxVec3 originalLinearVelocity;
    float maxContactImpulse;
    physx_PxVec3 originalAngularVelocity;
    float penBiasClamp;
    float invMass;
    uint32_t nodeIndex;
    float reportThreshold;
    uint32_t pad;
} physx_PxTGSSolverBodyData;
typedef struct physx_PxTGSSolverConstraintPrepDescBase {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxTGSSolverBodyVel const* body0;
    physx_PxTGSSolverBodyVel const* body1;
    physx_PxTGSSolverBodyTxInertia const* body0TxI;
    physx_PxTGSSolverBodyTxInertia const* body1TxI;
    physx_PxTGSSolverBodyData const* bodyData0;
    physx_PxTGSSolverBodyData const* bodyData1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    char structgen_pad0[8];
} physx_PxTGSSolverConstraintPrepDescBase;
typedef struct physx_PxTGSSolverConstraintPrepDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxTGSSolverBodyVel const* body0;
    physx_PxTGSSolverBodyVel const* body1;
    physx_PxTGSSolverBodyTxInertia const* body0TxI;
    physx_PxTGSSolverBodyTxInertia const* body1TxI;
    physx_PxTGSSolverBodyData const* bodyData0;
    physx_PxTGSSolverBodyData const* bodyData1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    physx_Px1DConstraint* rows;
    uint32_t numRows;
    float linBreakForce;
    float angBreakForce;
    float minResponseThreshold;
    void* writeback;
    bool disablePreprocessing;
    bool improvedSlerp;
    bool driveLimitsAreForces;
    bool extendedLimits;
    bool disableConstraint;
    char structgen_pad0[3];
    physx_PxVec3Padded body0WorldOffset;
    physx_PxVec3Padded cA2w;
    physx_PxVec3Padded cB2w;
} physx_PxTGSSolverConstraintPrepDesc;
typedef struct physx_PxTGSSolverContactDesc {
    physx_PxConstraintInvMassScale invMassScales;
    physx_PxSolverConstraintDesc* desc;
    physx_PxTGSSolverBodyVel const* body0;
    physx_PxTGSSolverBodyVel const* body1;
    physx_PxTGSSolverBodyTxInertia const* body0TxI;
    physx_PxTGSSolverBodyTxInertia const* body1TxI;
    physx_PxTGSSolverBodyData const* bodyData0;
    physx_PxTGSSolverBodyData const* bodyData1;
    physx_PxTransform bodyFrame0;
    physx_PxTransform bodyFrame1;
    int32_t bodyState0;
    int32_t bodyState1;
    void* shapeInteraction;
    physx_PxContactPoint* contacts;
    uint32_t numContacts;
    bool hasMaxImpulse;
    bool disableStrongFriction;
    bool hasForceThresholds;
    char structgen_pad0[1];
    float restDistance;
    float maxCCDSeparation;
    uint8_t* frictionPtr;
    uint8_t frictionCount;
    char structgen_pad1[7];
    float* contactForces;
    uint32_t startFrictionPatchIndex;
    uint32_t numFrictionPatches;
    uint32_t startContactPatchIndex;
    uint16_t numContactPatches;
    uint16_t axisConstraintCount;
    float maxImpulse;
    float torsionalPatchRadius;
    float minTorsionalPatchRadius;
    float offsetSlop;
} physx_PxTGSSolverContactDesc;
typedef struct physx_PxArticulationTendonLimit {
    float lowLimit;
    float highLimit;
} physx_PxArticulationTendonLimit;
typedef struct physx_PxArticulationAttachment {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationAttachment;
typedef struct physx_PxArticulationTendonJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationTendonJoint;
typedef struct physx_PxArticulationTendon {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationTendon;
typedef struct physx_PxArticulationSpatialTendon {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationSpatialTendon;
typedef struct physx_PxArticulationFixedTendon {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationFixedTendon;
typedef struct physx_PxSpatialForce {
    physx_PxVec3 force;
    float pad0;
    physx_PxVec3 torque;
    float pad1;
} physx_PxSpatialForce;
typedef struct physx_PxSpatialVelocity {
    physx_PxVec3 linear;
    float pad0;
    physx_PxVec3 angular;
    float pad1;
} physx_PxSpatialVelocity;
typedef struct physx_PxArticulationRootLinkData {
    physx_PxTransform transform;
    physx_PxVec3 worldLinVel;
    physx_PxVec3 worldAngVel;
    physx_PxVec3 worldLinAccel;
    physx_PxVec3 worldAngAccel;
} physx_PxArticulationRootLinkData;
typedef struct physx_PxArticulationCache {
    physx_PxSpatialForce* externalForces;
    float* denseJacobian;
    float* massMatrix;
    float* jointVelocity;
    float* jointAcceleration;
    float* jointPosition;
    float* jointForce;
    float* jointSolverForces;
    physx_PxSpatialVelocity* linkVelocity;
    physx_PxSpatialVelocity* linkAcceleration;
    physx_PxArticulationRootLinkData* rootLinkData;
    physx_PxSpatialForce* sensorForces;
    float* coefficientMatrix;
    float* lambda;
    void* scratchMemory;
    void* scratchAllocator;
    uint32_t version;
    char structgen_pad0[4];
} physx_PxArticulationCache;
typedef struct physx_PxArticulationSensor {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationSensor;
typedef struct physx_PxArticulationReducedCoordinate {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationReducedCoordinate;
typedef struct physx_PxArticulationJointReducedCoordinate {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationJointReducedCoordinate;
typedef struct physx_PxShape {
    char structgen_pad0[16];
    void* userData;
} physx_PxShape;
typedef struct physx_PxRigidActor {
    char structgen_pad0[16];
    void* userData;
} physx_PxRigidActor;
typedef struct physx_PxNodeIndex {
    char structgen_pad0[8];
} physx_PxNodeIndex;
typedef struct physx_PxRigidBody {
    char structgen_pad0[16];
    void* userData;
} physx_PxRigidBody;
typedef struct physx_PxArticulationLink {
    char structgen_pad0[16];
    void* userData;
} physx_PxArticulationLink;
typedef struct physx_PxConeLimitedConstraint {
    physx_PxVec3 mAxis;
    float mAngle;
    float mLowLimit;
    float mHighLimit;
} physx_PxConeLimitedConstraint;
typedef struct physx_PxConeLimitParams {
    physx_PxVec4 lowHighLimits;
    physx_PxVec4 axisAngle;
} physx_PxConeLimitParams;
typedef struct physx_PxConstraintShaderTable {
    void * solverPrep;
    char structgen_pad0[8];
    void * visualize;
    int32_t flag;
    char structgen_pad1[4];
} physx_PxConstraintShaderTable;
typedef struct physx_PxConstraint {
    char structgen_pad0[16];
    void* userData;
} physx_PxConstraint;
typedef struct physx_PxMassModificationProps {
    float mInvMassScale0;
    float mInvInertiaScale0;
    float mInvMassScale1;
    float mInvInertiaScale1;
} physx_PxMassModificationProps;
typedef struct physx_PxContactPatch {
    physx_PxMassModificationProps mMassModification;
    physx_PxVec3 normal;
    float restitution;
    float dynamicFriction;
    float staticFriction;
    float damping;
    uint16_t startContactIndex;
    uint8_t nbContacts;
    uint8_t materialFlags;
    uint16_t internalFlags;
    uint16_t materialIndex0;
    uint16_t materialIndex1;
    uint16_t pad[5];
} physx_PxContactPatch;
typedef struct physx_PxContact {
    physx_PxVec3 contact;
    float separation;
} physx_PxContact;
typedef struct physx_PxExtendedContact {
    physx_PxVec3 contact;
    float separation;
    physx_PxVec3 targetVelocity;
    float maxImpulse;
} physx_PxExtendedContact;
typedef struct physx_PxModifiableContact {
    physx_PxVec3 contact;
    float separation;
    physx_PxVec3 targetVelocity;
    float maxImpulse;
    physx_PxVec3 normal;
    float restitution;
    uint32_t materialFlags;
    uint16_t materialIndex0;
    uint16_t materialIndex1;
    float staticFriction;
    float dynamicFriction;
} physx_PxModifiableContact;
typedef struct physx_PxContactStreamIterator {
    physx_PxVec3 zero;
    char structgen_pad0[4];
    physx_PxContactPatch const* patch;
    physx_PxContact const* contact;
    uint32_t const* faceIndice;
    uint32_t totalPatches;
    uint32_t totalContacts;
    uint32_t nextContactIndex;
    uint32_t nextPatchIndex;
    uint32_t contactPatchHeaderSize;
    uint32_t contactPointSize;
    int32_t mStreamFormat;
    uint32_t forceNoResponse;
    bool pointStepped;
    char structgen_pad1[3];
    uint32_t hasFaceIndices;
} physx_PxContactStreamIterator;
typedef struct physx_PxGpuContactPair {
    uint8_t* contactPatches;
    uint8_t* contactPoints;
    float* contactForces;
    uint32_t transformCacheRef0;
    uint32_t transformCacheRef1;
    physx_PxNodeIndex nodeIndex0;
    physx_PxNodeIndex nodeIndex1;
    physx_PxActor* actor0;
    physx_PxActor* actor1;
    uint16_t nbContacts;
    uint16_t nbPatches;
    char structgen_pad0[4];
} physx_PxGpuContactPair;
typedef struct physx_PxContactSet {
    char structgen_pad0[16];
} physx_PxContactSet;
typedef struct physx_PxContactModifyPair {
    physx_PxRigidActor const* actor[2];
    physx_PxShape const* shape[2];
    physx_PxTransform transform[2];
    physx_PxContactSet contacts;
} physx_PxContactModifyPair;
typedef struct physx_PxBaseMaterial {
    char structgen_pad0[16];
    void* userData;
} physx_PxBaseMaterial;
typedef struct physx_PxFEMMaterial {
    char structgen_pad0[16];
    void* userData;
} physx_PxFEMMaterial;
typedef struct physx_PxFilterData {
    uint32_t word0;
    uint32_t word1;
    uint32_t word2;
    uint32_t word3;
} physx_PxFilterData;
typedef struct physx_PxParticleRigidFilterPair {
    uint64_t mID0;
    uint64_t mID1;
} physx_PxParticleRigidFilterPair;
typedef struct physx_PxMaterial {
    char structgen_pad0[16];
    void* userData;
} physx_PxMaterial;
typedef struct physx_PxGpuParticleBufferIndexPair {
    uint32_t systemIndex;
    uint32_t bufferIndex;
} physx_PxGpuParticleBufferIndexPair;
typedef struct physx_PxParticleVolume {
    physx_PxBounds3 bound;
    uint32_t particleIndicesOffset;
    uint32_t numParticles;
} physx_PxParticleVolume;
typedef struct physx_PxDiffuseParticleParams {
    float threshold;
    float lifetime;
    float airDrag;
    float bubbleDrag;
    float buoyancy;
    float kineticEnergyWeight;
    float pressureWeight;
    float divergenceWeight;
    float collisionDecay;
    bool useAccurateVelocity;
    char structgen_pad0[3];
} physx_PxDiffuseParticleParams;
typedef struct physx_PxParticleSpring {
    uint32_t ind0;
    uint32_t ind1;
    float length;
    float stiffness;
    float damping;
    float pad;
} physx_PxParticleSpring;
typedef struct physx_PxParticleMaterial {
    char structgen_pad0[16];
    void* userData;
} physx_PxParticleMaterial;
typedef struct physx_PxActorShape {
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
} physx_PxActorShape;
typedef struct physx_PxRaycastHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
    float u;
    float v;
    char structgen_pad1[4];
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
} physx_PxRaycastHit;
typedef struct physx_PxOverlapHit {
    uint32_t faceIndex;
    char structgen_pad0[4];
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
} physx_PxOverlapHit;
typedef struct physx_PxSweepHit {
    uint32_t faceIndex;
    uint16_t flags;
    char structgen_pad0[2];
    physx_PxVec3 position;
    physx_PxVec3 normal;
    float distance;
    char structgen_pad1[4];
    physx_PxRigidActor* actor;
    physx_PxShape* shape;
} physx_PxSweepHit;
typedef struct physx_PxRaycastCallback {
    char structgen_pad0[8];
    physx_PxRaycastHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxRaycastHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
} physx_PxRaycastCallback;
typedef struct physx_PxOverlapCallback {
    char structgen_pad0[8];
    physx_PxOverlapHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxOverlapHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
} physx_PxOverlapCallback;
typedef struct physx_PxSweepCallback {
    char structgen_pad0[8];
    physx_PxSweepHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxSweepHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
} physx_PxSweepCallback;
typedef struct physx_PxRaycastBuffer {
    char structgen_pad0[8];
    physx_PxRaycastHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxRaycastHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
} physx_PxRaycastBuffer;
typedef struct physx_PxOverlapBuffer {
    char structgen_pad0[8];
    physx_PxOverlapHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxOverlapHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
} physx_PxOverlapBuffer;
typedef struct physx_PxSweepBuffer {
    char structgen_pad0[8];
    physx_PxSweepHit block;
    bool hasBlock;
    char structgen_pad1[7];
    physx_PxSweepHit* touches;
    uint32_t maxNbTouches;
    uint32_t nbTouches;
} physx_PxSweepBuffer;
typedef struct physx_PxQueryCache {
    physx_PxShape* shape;
    physx_PxRigidActor* actor;
    uint32_t faceIndex;
    char structgen_pad0[4];
} physx_PxQueryCache;
typedef struct physx_PxQueryFilterData {
    physx_PxFilterData data;
    uint16_t flags;
    char structgen_pad0[2];
} physx_PxQueryFilterData;
typedef struct physx_PxRigidDynamic {
    char structgen_pad0[16];
    void* userData;
} physx_PxRigidDynamic;
typedef struct physx_PxRigidStatic {
    char structgen_pad0[16];
    void* userData;
} physx_PxRigidStatic;
typedef struct physx_PxSceneQueryDesc {
    int32_t staticStructure;
    int32_t dynamicStructure;
    uint32_t dynamicTreeRebuildRateHint;
    int32_t dynamicTreeSecondaryPruner;
    int32_t staticBVHBuildStrategy;
    int32_t dynamicBVHBuildStrategy;
    uint32_t staticNbObjectsPerNode;
    uint32_t dynamicNbObjectsPerNode;
    int32_t sceneQueryUpdateMode;
} physx_PxSceneQueryDesc;
typedef struct physx_PxBroadPhaseRegion {
    physx_PxBounds3 mBounds;
    void* mUserData;
} physx_PxBroadPhaseRegion;
typedef struct physx_PxBroadPhaseRegionInfo {
    physx_PxBroadPhaseRegion mRegion;
    uint32_t mNbStaticObjects;
    uint32_t mNbDynamicObjects;
    bool mActive;
    bool mOverlap;
    char structgen_pad0[6];
} physx_PxBroadPhaseRegionInfo;
typedef struct physx_PxBroadPhaseCaps {
    uint32_t mMaxNbRegions;
} physx_PxBroadPhaseCaps;
typedef struct physx_PxBroadPhaseDesc {
    int32_t mType;
    char structgen_pad0[4];
    uint64_t mContextID;
    char structgen_pad1[8];
    uint32_t mFoundLostPairsCapacity;
    bool mDiscardStaticVsKinematic;
    bool mDiscardKinematicVsKinematic;
    char structgen_pad2[2];
} physx_PxBroadPhaseDesc;
typedef struct physx_PxBroadPhaseUpdateData {
    uint32_t const* mCreated;
    uint32_t mNbCreated;
    char structgen_pad0[4];
    uint32_t const* mUpdated;
    uint32_t mNbUpdated;
    char structgen_pad1[4];
    uint32_t const* mRemoved;
    uint32_t mNbRemoved;
    char structgen_pad2[4];
    physx_PxBounds3 const* mBounds;
    uint32_t const* mGroups;
    float const* mDistances;
    uint32_t mCapacity;
    char structgen_pad3[4];
} physx_PxBroadPhaseUpdateData;
typedef struct physx_PxBroadPhasePair {
    uint32_t mID0;
    uint32_t mID1;
} physx_PxBroadPhasePair;
typedef struct physx_PxBroadPhaseResults {
    uint32_t mNbCreatedPairs;
    char structgen_pad0[4];
    physx_PxBroadPhasePair const* mCreatedPairs;
    uint32_t mNbDeletedPairs;
    char structgen_pad1[4];
    physx_PxBroadPhasePair const* mDeletedPairs;
} physx_PxBroadPhaseResults;
typedef struct physx_PxSceneLimits {
    uint32_t maxNbActors;
    uint32_t maxNbBodies;
    uint32_t maxNbStaticShapes;
    uint32_t maxNbDynamicShapes;
    uint32_t maxNbAggregates;
    uint32_t maxNbConstraints;
    uint32_t maxNbRegions;
    uint32_t maxNbBroadPhaseOverlaps;
} physx_PxSceneLimits;
typedef struct physx_PxgDynamicsMemoryConfig {
    uint32_t tempBufferCapacity;
    uint32_t maxRigidContactCount;
    uint32_t maxRigidPatchCount;
    uint32_t heapCapacity;
    uint32_t foundLostPairsCapacity;
    uint32_t foundLostAggregatePairsCapacity;
    uint32_t totalAggregatePairsCapacity;
    uint32_t maxSoftBodyContacts;
    uint32_t maxFemClothContacts;
    uint32_t maxParticleContacts;
    uint32_t collisionStackSize;
    uint32_t maxHairContacts;
} physx_PxgDynamicsMemoryConfig;
typedef struct physx_PxSceneDesc {
    int32_t staticStructure;
    int32_t dynamicStructure;
    uint32_t dynamicTreeRebuildRateHint;
    int32_t dynamicTreeSecondaryPruner;
    int32_t staticBVHBuildStrategy;
    int32_t dynamicBVHBuildStrategy;
    uint32_t staticNbObjectsPerNode;
    uint32_t dynamicNbObjectsPerNode;
    int32_t sceneQueryUpdateMode;
    physx_PxVec3 gravity;
    physx_PxSimulationEventCallback* simulationEventCallback;
    physx_PxContactModifyCallback* contactModifyCallback;
    physx_PxCCDContactModifyCallback* ccdContactModifyCallback;
    void const* filterShaderData;
    uint32_t filterShaderDataSize;
    char structgen_pad0[4];
    void * filterShader;
    physx_PxSimulationFilterCallback* filterCallback;
    int32_t kineKineFilteringMode;
    int32_t staticKineFilteringMode;
    int32_t broadPhaseType;
    char structgen_pad1[4];
    physx_PxBroadPhaseCallback* broadPhaseCallback;
    physx_PxSceneLimits limits;
    int32_t frictionType;
    int32_t solverType;
    float bounceThresholdVelocity;
    float frictionOffsetThreshold;
    float frictionCorrelationDistance;
    uint32_t flags;
    physx_PxCpuDispatcher* cpuDispatcher;
    char structgen_pad2[8];
    void* userData;
    uint32_t solverBatchSize;
    uint32_t solverArticulationBatchSize;
    uint32_t nbContactDataBlocks;
    uint32_t maxNbContactDataBlocks;
    float maxBiasCoefficient;
    uint32_t contactReportStreamBufferSize;
    uint32_t ccdMaxPasses;
    float ccdThreshold;
    float ccdMaxSeparation;
    float wakeCounterResetValue;
    physx_PxBounds3 sanityBounds;
    physx_PxgDynamicsMemoryConfig gpuDynamicsConfig;
    uint32_t gpuMaxNumPartitions;
    uint32_t gpuMaxNumStaticPartitions;
    uint32_t gpuComputeVersion;
    uint32_t contactPairSlabSize;
    physx_PxSceneQuerySystem* sceneQuerySystem;
    char structgen_pad3[8];
} physx_PxSceneDesc;
typedef struct physx_PxSimulationStatistics {
    uint32_t nbActiveConstraints;
    uint32_t nbActiveDynamicBodies;
    uint32_t nbActiveKinematicBodies;
    uint32_t nbStaticBodies;
    uint32_t nbDynamicBodies;
    uint32_t nbKinematicBodies;
    uint32_t nbShapes[11];
    uint32_t nbAggregates;
    uint32_t nbArticulations;
    uint32_t nbAxisSolverConstraints;
    uint32_t compressedContactSize;
    uint32_t requiredContactConstraintMemory;
    uint32_t peakConstraintMemory;
    uint32_t nbDiscreteContactPairsTotal;
    uint32_t nbDiscreteContactPairsWithCacheHits;
    uint32_t nbDiscreteContactPairsWithContacts;
    uint32_t nbNewPairs;
    uint32_t nbLostPairs;
    uint32_t nbNewTouches;
    uint32_t nbLostTouches;
    uint32_t nbPartitions;
    char structgen_pad0[4];
    uint64_t gpuMemParticles;
    uint64_t gpuMemSoftBodies;
    uint64_t gpuMemFEMCloths;
    uint64_t gpuMemHairSystems;
    uint64_t gpuMemHeap;
    uint64_t gpuMemHeapBroadPhase;
    uint64_t gpuMemHeapNarrowPhase;
    uint64_t gpuMemHeapSolver;
    uint64_t gpuMemHeapArticulation;
    uint64_t gpuMemHeapSimulation;
    uint64_t gpuMemHeapSimulationArticulation;
    uint64_t gpuMemHeapSimulationParticles;
    uint64_t gpuMemHeapSimulationSoftBody;
    uint64_t gpuMemHeapSimulationFEMCloth;
    uint64_t gpuMemHeapSimulationHairSystem;
    uint64_t gpuMemHeapParticles;
    uint64_t gpuMemHeapSoftBodies;
    uint64_t gpuMemHeapFEMCloths;
    uint64_t gpuMemHeapHairSystems;
    uint64_t gpuMemHeapOther;
    uint32_t nbBroadPhaseAdds;
    uint32_t nbBroadPhaseRemoves;
    uint32_t nbDiscreteContactPairs[11][11];
    uint32_t nbCCDPairs[11][11];
    uint32_t nbModifiedContactPairs[11][11];
    uint32_t nbTriggerPairs[11][11];
} physx_PxSimulationStatistics;
typedef struct physx_PxGpuBodyData {
    physx_PxQuat quat;
    physx_PxVec4 pos;
    physx_PxVec4 linVel;
    physx_PxVec4 angVel;
} physx_PxGpuBodyData;
typedef struct physx_PxGpuActorPair {
    uint32_t srcIndex;
    char structgen_pad0[4];
    physx_PxNodeIndex nodeIndex;
} physx_PxGpuActorPair;
typedef struct physx_PxIndexDataPair {
    uint32_t index;
    char structgen_pad0[4];
    void* data;
} physx_PxIndexDataPair;
typedef struct physx_PxDominanceGroupPair {
    uint8_t dominance0;
    uint8_t dominance1;
} physx_PxDominanceGroupPair;
typedef struct physx_PxScene {
    char structgen_pad0[8];
    void* userData;
} physx_PxScene;
typedef struct physx_PxSceneReadLock {
    char structgen_pad0[8];
} physx_PxSceneReadLock;
typedef struct physx_PxSceneWriteLock {
    char structgen_pad0[8];
} physx_PxSceneWriteLock;
typedef struct physx_PxContactPairExtraDataItem {
    uint8_t type_;
} physx_PxContactPairExtraDataItem;
typedef struct physx_PxContactPairVelocity {
    uint8_t type_;
    char structgen_pad0[3];
    physx_PxVec3 linearVelocity[2];
    physx_PxVec3 angularVelocity[2];
} physx_PxContactPairVelocity;
typedef struct physx_PxContactPairPose {
    uint8_t type_;
    char structgen_pad0[3];
    physx_PxTransform globalPose[2];
} physx_PxContactPairPose;
typedef struct physx_PxContactPairIndex {
    uint8_t type_;
    char structgen_pad0[1];
    uint16_t index;
} physx_PxContactPairIndex;
typedef struct physx_PxContactPairExtraDataIterator {
    uint8_t const* currPtr;
    uint8_t const* endPtr;
    physx_PxContactPairVelocity const* preSolverVelocity;
    physx_PxContactPairVelocity const* postSolverVelocity;
    physx_PxContactPairPose const* eventPose;
    uint32_t contactPairIndex;
    char structgen_pad0[4];
} physx_PxContactPairExtraDataIterator;
typedef struct physx_PxContactPairHeader {
    physx_PxActor* actors[2];
    uint8_t const* extraDataStream;
    uint16_t extraDataStreamSize;
    uint16_t flags;
    char structgen_pad0[4];
    physx_PxContactPair const* pairs;
    uint32_t nbPairs;
    char structgen_pad1[4];
} physx_PxContactPairHeader;
typedef struct physx_PxContactPairPoint {
    physx_PxVec3 position;
    float separation;
    physx_PxVec3 normal;
    uint32_t internalFaceIndex0;
    physx_PxVec3 impulse;
    uint32_t internalFaceIndex1;
} physx_PxContactPairPoint;
typedef struct physx_PxContactPair {
    physx_PxShape* shapes[2];
    uint8_t const* contactPatches;
    uint8_t const* contactPoints;
    float const* contactImpulses;
    uint32_t requiredBufferSize;
    uint8_t contactCount;
    uint8_t patchCount;
    uint16_t contactStreamSize;
    uint16_t flags;
    uint16_t events;
    uint32_t internalData[2];
    char structgen_pad0[4];
} physx_PxContactPair;
typedef struct physx_PxTriggerPair {
    physx_PxShape* triggerShape;
    physx_PxActor* triggerActor;
    physx_PxShape* otherShape;
    physx_PxActor* otherActor;
    int32_t status;
    uint8_t flags;
    char structgen_pad0[3];
} physx_PxTriggerPair;
typedef struct physx_PxConstraintInfo {
    physx_PxConstraint* constraint;
    void* externalReference;
    uint32_t type_;
    char structgen_pad0[4];
} physx_PxConstraintInfo;
typedef struct physx_PxFEMParameters {
    float velocityDamping;
    float settlingThreshold;
    float sleepThreshold;
    float sleepDamping;
    float selfCollisionFilterDistance;
    float selfCollisionStressTolerance;
} physx_PxFEMParameters;
typedef struct physx_PxPruningStructure {
    char structgen_pad0[16];
} physx_PxPruningStructure;
typedef struct physx_PxExtendedVec3 {
    double x;
    double y;
    double z;
} physx_PxExtendedVec3;
typedef struct physx_PxObstacle {
    char structgen_pad0[8];
    void* mUserData;
    physx_PxExtendedVec3 mPos;
    physx_PxQuat mRot;
} physx_PxObstacle;
typedef struct physx_PxBoxObstacle {
    char structgen_pad0[8];
    void* mUserData;
    physx_PxExtendedVec3 mPos;
    physx_PxQuat mRot;
    physx_PxVec3 mHalfExtents;
    char structgen_pad1[4];
} physx_PxBoxObstacle;
typedef struct physx_PxCapsuleObstacle {
    char structgen_pad0[8];
    void* mUserData;
    physx_PxExtendedVec3 mPos;
    physx_PxQuat mRot;
    float mHalfHeight;
    float mRadius;
} physx_PxCapsuleObstacle;
typedef struct physx_PxControllerState {
    physx_PxVec3 deltaXP;
    char structgen_pad0[4];
    physx_PxShape* touchedShape;
    physx_PxRigidActor* touchedActor;
    uint32_t touchedObstacleHandle;
    uint32_t collisionFlags;
    bool standOnAnotherCCT;
    bool standOnObstacle;
    bool isMovingUp;
    char structgen_pad1[5];
} physx_PxControllerState;
typedef struct physx_PxControllerStats {
    uint16_t nbIterations;
    uint16_t nbFullUpdates;
    uint16_t nbPartialUpdates;
    uint16_t nbTessellation;
} physx_PxControllerStats;
typedef struct physx_PxControllerHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
} physx_PxControllerHit;
typedef struct physx_PxControllerShapeHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
    physx_PxShape* shape;
    physx_PxRigidActor* actor;
    uint32_t triangleIndex;
    char structgen_pad1[4];
} physx_PxControllerShapeHit;
typedef struct physx_PxControllersHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
    physx_PxController* other;
} physx_PxControllersHit;
typedef struct physx_PxControllerObstacleHit {
    physx_PxController* controller;
    physx_PxExtendedVec3 worldPos;
    physx_PxVec3 worldNormal;
    physx_PxVec3 dir;
    float length;
    char structgen_pad0[4];
    void const* userData;
} physx_PxControllerObstacleHit;
typedef struct physx_PxControllerFilters {
    physx_PxFilterData const* mFilterData;
    physx_PxQueryFilterCallback* mFilterCallback;
    uint16_t mFilterFlags;
    char structgen_pad0[6];
    physx_PxControllerFilterCallback* mCCTFilterCallback;
} physx_PxControllerFilters;
typedef struct physx_PxControllerDesc {
    char structgen_pad0[8];
    physx_PxExtendedVec3 position;
    physx_PxVec3 upDirection;
    float slopeLimit;
    float invisibleWallHeight;
    float maxJumpHeight;
    float contactOffset;
    float stepOffset;
    float density;
    float scaleCoeff;
    float volumeGrowth;
    char structgen_pad1[4];
    physx_PxUserControllerHitReport* reportCallback;
    physx_PxControllerBehaviorCallback* behaviorCallback;
    int32_t nonWalkableMode;
    char structgen_pad2[4];
    physx_PxMaterial* material;
    bool registerDeletionListener;
    uint8_t clientID;
    char structgen_pad3[6];
    void* userData;
    char structgen_pad4[8];
} physx_PxControllerDesc;
typedef struct physx_PxBoxControllerDesc {
    char structgen_pad0[8];
    physx_PxExtendedVec3 position;
    physx_PxVec3 upDirection;
    float slopeLimit;
    float invisibleWallHeight;
    float maxJumpHeight;
    float contactOffset;
    float stepOffset;
    float density;
    float scaleCoeff;
    float volumeGrowth;
    char structgen_pad1[4];
    physx_PxUserControllerHitReport* reportCallback;
    physx_PxControllerBehaviorCallback* behaviorCallback;
    int32_t nonWalkableMode;
    char structgen_pad2[4];
    physx_PxMaterial* material;
    bool registerDeletionListener;
    uint8_t clientID;
    char structgen_pad3[6];
    void* userData;
    char structgen_pad4[4];
    float halfHeight;
    float halfSideExtent;
    float halfForwardExtent;
} physx_PxBoxControllerDesc;
typedef struct physx_PxCapsuleControllerDesc {
    char structgen_pad0[8];
    physx_PxExtendedVec3 position;
    physx_PxVec3 upDirection;
    float slopeLimit;
    float invisibleWallHeight;
    float maxJumpHeight;
    float contactOffset;
    float stepOffset;
    float density;
    float scaleCoeff;
    float volumeGrowth;
    char structgen_pad1[4];
    physx_PxUserControllerHitReport* reportCallback;
    physx_PxControllerBehaviorCallback* behaviorCallback;
    int32_t nonWalkableMode;
    char structgen_pad2[4];
    physx_PxMaterial* material;
    bool registerDeletionListener;
    uint8_t clientID;
    char structgen_pad3[6];
    void* userData;
    char structgen_pad4[4];
    float radius;
    float height;
    int32_t climbingMode;
} physx_PxCapsuleControllerDesc;
typedef struct physx_PxDim3 {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} physx_PxDim3;
typedef struct physx_PxSDFDesc {
    physx_PxBoundedData sdf;
    physx_PxDim3 dims;
    physx_PxVec3 meshLower;
    float spacing;
    uint32_t subgridSize;
    int32_t bitsPerSubgridPixel;
    physx_PxDim3 sdfSubgrids3DTexBlockDim;
    physx_PxBoundedData sdfSubgrids;
    physx_PxBoundedData sdfStartSlots;
    float subgridsMinSdfValue;
    float subgridsMaxSdfValue;
    physx_PxBounds3 sdfBounds;
    float narrowBandThicknessRelativeToSdfBoundsDiagonal;
    uint32_t numThreadsForSdfConstruction;
} physx_PxSDFDesc;
typedef struct physx_PxConvexMeshDesc {
    physx_PxBoundedData points;
    physx_PxBoundedData polygons;
    physx_PxBoundedData indices;
    uint16_t flags;
    uint16_t vertexLimit;
    uint16_t polygonLimit;
    uint16_t quantizedCount;
    physx_PxSDFDesc* sdfDesc;
} physx_PxConvexMeshDesc;
typedef struct physx_PxTriangleMeshDesc {
    physx_PxBoundedData points;
    physx_PxBoundedData triangles;
    uint16_t flags;
    char structgen_pad0[22];
    physx_PxSDFDesc* sdfDesc;
} physx_PxTriangleMeshDesc;
typedef struct physx_PxTetrahedronMeshDesc {
    char structgen_pad0[16];
    physx_PxBoundedData points;
    physx_PxBoundedData tetrahedrons;
    uint16_t flags;
    uint16_t tetsPerElement;
    char structgen_pad1[4];
} physx_PxTetrahedronMeshDesc;
typedef struct physx_PxSoftBodySimulationDataDesc {
    physx_PxBoundedData vertexToTet;
} physx_PxSoftBodySimulationDataDesc;
typedef struct physx_PxBVH34MidphaseDesc {
    uint32_t numPrimsPerLeaf;
    int32_t buildStrategy;
    bool quantized;
    char structgen_pad0[3];
} physx_PxBVH34MidphaseDesc;
typedef struct physx_PxMidphaseDesc {
    char structgen_pad0[16];
} physx_PxMidphaseDesc;
typedef struct physx_PxBVHDesc {
    physx_PxBoundedData bounds;
    float enlargement;
    uint32_t numPrimsPerLeaf;
    int32_t buildStrategy;
    char structgen_pad0[4];
} physx_PxBVHDesc;
typedef struct physx_PxCookingParams {
    float areaTestEpsilon;
    float planeTolerance;
    int32_t convexMeshCookingType;
    bool suppressTriangleMeshRemapTable;
    bool buildTriangleAdjacencies;
    bool buildGPUData;
    char structgen_pad0[1];
    physx_PxTolerancesScale scale;
    uint32_t meshPreprocessParams;
    float meshWeldTolerance;
    physx_PxMidphaseDesc midphaseDesc;
    uint32_t gaussMapLimit;
    float maxWeightRatioInTet;
} physx_PxCookingParams;
typedef struct physx_PxDefaultMemoryOutputStream {
    char structgen_pad0[32];
} physx_PxDefaultMemoryOutputStream;
typedef struct physx_PxDefaultMemoryInputData {
    char structgen_pad0[32];
} physx_PxDefaultMemoryInputData;
typedef struct physx_PxDefaultFileOutputStream {
    char structgen_pad0[16];
} physx_PxDefaultFileOutputStream;
typedef struct physx_PxDefaultFileInputData {
    char structgen_pad0[24];
} physx_PxDefaultFileInputData;
typedef struct physx_PxJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxJoint;
typedef struct physx_PxSpring {
    float stiffness;
    float damping;
} physx_PxSpring;
typedef struct physx_PxDistanceJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxDistanceJoint;
typedef struct physx_PxJacobianRow {
    physx_PxVec3 linear0;
    physx_PxVec3 linear1;
    physx_PxVec3 angular0;
    physx_PxVec3 angular1;
} physx_PxJacobianRow;
typedef struct physx_PxContactJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxContactJoint;
typedef struct physx_PxFixedJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxFixedJoint;
typedef struct physx_PxJointLimitParameters {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
} physx_PxJointLimitParameters;
typedef struct physx_PxJointLinearLimit {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float value;
} physx_PxJointLinearLimit;
typedef struct physx_PxJointLinearLimitPair {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float upper;
    float lower;
} physx_PxJointLinearLimitPair;
typedef struct physx_PxJointAngularLimitPair {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float upper;
    float lower;
} physx_PxJointAngularLimitPair;
typedef struct physx_PxJointLimitCone {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float yAngle;
    float zAngle;
} physx_PxJointLimitCone;
typedef struct physx_PxJointLimitPyramid {
    float restitution;
    float bounceThreshold;
    float stiffness;
    float damping;
    float contactDistance_deprecated;
    float yAngleMin;
    float yAngleMax;
    float zAngleMin;
    float zAngleMax;
} physx_PxJointLimitPyramid;
typedef struct physx_PxPrismaticJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxPrismaticJoint;
typedef struct physx_PxRevoluteJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxRevoluteJoint;
typedef struct physx_PxSphericalJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxSphericalJoint;
typedef struct physx_PxD6JointDrive {
    float stiffness;
    float damping;
    float forceLimit;
    uint32_t flags;
} physx_PxD6JointDrive;
typedef struct physx_PxD6Joint {
    char structgen_pad0[16];
    void* userData;
} physx_PxD6Joint;
typedef struct physx_PxGearJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxGearJoint;
typedef struct physx_PxRackAndPinionJoint {
    char structgen_pad0[16];
    void* userData;
} physx_PxRackAndPinionJoint;
typedef struct physx_PxGroupsMask {
    uint16_t bits0;
    uint16_t bits1;
    uint16_t bits2;
    uint16_t bits3;
} physx_PxGroupsMask;
typedef struct physx_PxRigidActorExt {
    char structgen_pad0[1];
} physx_PxRigidActorExt;
typedef struct physx_PxMassProperties {
    physx_PxMat33 inertiaTensor;
    physx_PxVec3 centerOfMass;
    float mass;
} physx_PxMassProperties;
typedef struct physx_PxRigidBodyExt {
    char structgen_pad0[1];
} physx_PxRigidBodyExt;
typedef struct physx_PxShapeExt {
    char structgen_pad0[1];
} physx_PxShapeExt;
typedef struct physx_PxMeshOverlapUtil {
    char structgen_pad0[1040];
} physx_PxMeshOverlapUtil;
typedef struct physx_PxXmlMiscParameter {
    physx_PxVec3 upVector;
    physx_PxTolerancesScale scale;
} physx_PxXmlMiscParameter;
typedef struct physx_PxSerialization {
    char structgen_pad0[1];
} physx_PxSerialization;
typedef struct physx_PxStringTableExt {
    char structgen_pad0[1];
} physx_PxStringTableExt;
typedef struct physx_PxBroadPhaseExt {
    char structgen_pad0[1];
} physx_PxBroadPhaseExt;
typedef struct physx_PxSceneQueryExt {
    char structgen_pad0[1];
} physx_PxSceneQueryExt;
typedef struct physx_PxSamplingExt {
    char structgen_pad0[1];
} physx_PxSamplingExt;
typedef struct physx_PxPoissonSampler {
    char structgen_pad0[8];
} physx_PxPoissonSampler;
typedef struct physx_PxTriangleMeshPoissonSampler {
    char structgen_pad0[8];
} physx_PxTriangleMeshPoissonSampler;
typedef struct physx_PxTetrahedronMeshExt {
    char structgen_pad0[1];
} physx_PxTetrahedronMeshExt;
typedef struct physx_PxRepXObject {
    char const* typeName;
    void const* serializable;
    uint64_t id;
} physx_PxRepXObject;
typedef struct physx_PxRepXInstantiationArgs {
    char structgen_pad0[8];
    physx_PxCooking* cooker;
    physx_PxStringTable* stringTable;
} physx_PxRepXInstantiationArgs;
        static_assert(sizeof(physx_PxAllocator) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxRawAllocator) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxVirtualAllocator) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxUserAllocated) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxTempAllocator) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxBitAndByte) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxBitMap) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxVec3) == 12, "assertion failed:");
        static_assert(sizeof(physx_PxVec3Padded) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxQuat) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxTransform) == 28, "assertion failed:");
        static_assert(sizeof(physx_PxTransformPadded) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxMat33) == 36, "assertion failed:");
        static_assert(sizeof(physx_PxBounds3) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxBroadcastingAllocator) == 176, "assertion failed:");
        static_assert(sizeof(physx_PxBroadcastingErrorCallback) == 160, "assertion failed:");
        static_assert(sizeof(physx_PxVec4) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxMat44) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxPlane) == 16, "assertion failed:");
        static_assert(sizeof(physx_Interpolation) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxMutexImpl) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxReadWriteLock) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxProfileScoped) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxSListEntry) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxSListImpl) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxSyncImpl) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxCounterFrequencyToTensOfNanos) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxTime) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxVec2) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxStridedData) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxBoundedData) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxDebugPoint) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxDebugLine) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxDebugTriangle) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxDebugText) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxDeserializationContext) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxBase) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxRefCounted) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxTolerancesScale) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxMetaDataEntry) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxBaseTask) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxTask) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxLightCpuTask) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxGeometry) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxBoxGeometry) == 20, "assertion failed:");
        static_assert(sizeof(physx_PxBVH) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxCapsuleGeometry) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxHullPolygon) == 20, "assertion failed:");
        static_assert(sizeof(physx_PxConvexMesh) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxMeshScale) == 28, "assertion failed:");
        static_assert(sizeof(physx_PxConvexMeshGeometry) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxSphereGeometry) == 12, "assertion failed:");
        static_assert(sizeof(physx_PxPlaneGeometry) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxTriangleMeshGeometry) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxHeightFieldGeometry) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxParticleSystemGeometry) == 12, "assertion failed:");
        static_assert(sizeof(physx_PxHairSystemGeometry) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxTetrahedronMeshGeometry) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxQueryHit) == 4, "assertion failed:");
        static_assert(sizeof(physx_PxLocationHit) == 36, "assertion failed:");
        static_assert(sizeof(physx_PxGeomRaycastHit) == 44, "assertion failed:");
        static_assert(sizeof(physx_PxGeomOverlapHit) == 4, "assertion failed:");
        static_assert(sizeof(physx_PxGeomSweepHit) == 36, "assertion failed:");
        static_assert(sizeof(physx_PxGeomIndexPair) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxQueryThreadContext) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxCustomGeometryType) == 4, "assertion failed:");
        static_assert(sizeof(physx_PxCustomGeometry) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxGeometryHolder) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxGeometryQuery) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxHeightFieldSample) == 4, "assertion failed:");
        static_assert(sizeof(physx_PxHeightField) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxHeightFieldDesc) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxMeshQuery) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxSimpleTriangleMesh) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxTriangle) == 36, "assertion failed:");
        static_assert(sizeof(physx_PxTrianglePadded) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxTriangleMesh) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxBVH34TriangleMesh) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxTetrahedron) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxSoftBodyAuxData) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxTetrahedronMesh) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxSoftBodyMesh) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxCollisionMeshMappingData) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxSoftBodyCollisionData) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxTetrahedronMeshData) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxSoftBodySimulationData) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxCollisionTetrahedronMeshData) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxSimulationTetrahedronMeshData) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxActor) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxAggregate) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxSpringModifiers) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxRestitutionModifiers) == 16, "assertion failed:");
        static_assert(sizeof(physx_Px1DConstraint) == 96, "assertion failed:");
        static_assert(sizeof(physx_PxConstraintInvMassScale) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxContactPoint) == 80, "assertion failed:");
        static_assert(sizeof(physx_PxSolverBody) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxSolverBodyData) == 112, "assertion failed:");
        static_assert(sizeof(physx_PxConstraintBatchHeader) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxSolverConstraintDesc) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxSolverConstraintPrepDescBase) == 128, "assertion failed:");
        static_assert(sizeof(physx_PxSolverConstraintPrepDesc) == 192, "assertion failed:");
        static_assert(sizeof(physx_PxSolverContactDesc) == 208, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationLimit) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationDrive) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxTGSSolverBodyVel) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxTGSSolverBodyTxInertia) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxTGSSolverBodyData) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxTGSSolverConstraintPrepDescBase) == 144, "assertion failed:");
        static_assert(sizeof(physx_PxTGSSolverConstraintPrepDesc) == 224, "assertion failed:");
        static_assert(sizeof(physx_PxTGSSolverContactDesc) == 224, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationTendonLimit) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationAttachment) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationTendonJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationTendon) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationSpatialTendon) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationFixedTendon) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxSpatialForce) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxSpatialVelocity) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationRootLinkData) == 76, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationCache) == 136, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationSensor) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationReducedCoordinate) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationJointReducedCoordinate) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxShape) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxRigidActor) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxNodeIndex) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxRigidBody) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxArticulationLink) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxConeLimitedConstraint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxConeLimitParams) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxConstraintShaderTable) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxConstraint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxMassModificationProps) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxContactPatch) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxContact) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxExtendedContact) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxModifiableContact) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxContactStreamIterator) == 80, "assertion failed:");
        static_assert(sizeof(physx_PxGpuContactPair) == 72, "assertion failed:");
        static_assert(sizeof(physx_PxContactSet) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxContactModifyPair) == 104, "assertion failed:");
        static_assert(sizeof(physx_PxBaseMaterial) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxFEMMaterial) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxFilterData) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxParticleRigidFilterPair) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxMaterial) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxGpuParticleBufferIndexPair) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxParticleVolume) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxDiffuseParticleParams) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxParticleSpring) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxParticleMaterial) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxActorShape) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxRaycastHit) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxOverlapHit) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxSweepHit) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxRaycastCallback) == 96, "assertion failed:");
        static_assert(sizeof(physx_PxOverlapCallback) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxSweepCallback) == 88, "assertion failed:");
        static_assert(sizeof(physx_PxRaycastBuffer) == 96, "assertion failed:");
        static_assert(sizeof(physx_PxOverlapBuffer) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxSweepBuffer) == 88, "assertion failed:");
        static_assert(sizeof(physx_PxQueryCache) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxQueryFilterData) == 20, "assertion failed:");
        static_assert(sizeof(physx_PxRigidDynamic) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxRigidStatic) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxSceneQueryDesc) == 36, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseRegion) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseRegionInfo) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseCaps) == 4, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseDesc) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseUpdateData) == 80, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhasePair) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseResults) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxSceneLimits) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxgDynamicsMemoryConfig) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxSceneDesc) == 352, "assertion failed:");
        static_assert(sizeof(physx_PxSimulationStatistics) == 2232, "assertion failed:");
        static_assert(sizeof(physx_PxGpuBodyData) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxGpuActorPair) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxIndexDataPair) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxDominanceGroupPair) == 2, "assertion failed:");
        static_assert(sizeof(physx_PxScene) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxSceneReadLock) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxSceneWriteLock) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairExtraDataItem) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairVelocity) == 52, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairPose) == 60, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairIndex) == 4, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairExtraDataIterator) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairHeader) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxContactPairPoint) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxContactPair) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxTriggerPair) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxConstraintInfo) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxFEMParameters) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxPruningStructure) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxExtendedVec3) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxObstacle) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxBoxObstacle) == 72, "assertion failed:");
        static_assert(sizeof(physx_PxCapsuleObstacle) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxControllerState) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxControllerStats) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxControllerHit) == 64, "assertion failed:");
        static_assert(sizeof(physx_PxControllerShapeHit) == 88, "assertion failed:");
        static_assert(sizeof(physx_PxControllersHit) == 72, "assertion failed:");
        static_assert(sizeof(physx_PxControllerObstacleHit) == 72, "assertion failed:");
        static_assert(sizeof(physx_PxControllerFilters) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxControllerDesc) == 136, "assertion failed:");
        static_assert(sizeof(physx_PxBoxControllerDesc) == 144, "assertion failed:");
        static_assert(sizeof(physx_PxCapsuleControllerDesc) == 144, "assertion failed:");
        static_assert(sizeof(physx_PxDim3) == 12, "assertion failed:");
        static_assert(sizeof(physx_PxSDFDesc) == 160, "assertion failed:");
        static_assert(sizeof(physx_PxConvexMeshDesc) == 88, "assertion failed:");
        static_assert(sizeof(physx_PxTriangleMeshDesc) == 80, "assertion failed:");
        static_assert(sizeof(physx_PxTetrahedronMeshDesc) == 72, "assertion failed:");
        static_assert(sizeof(physx_PxSoftBodySimulationDataDesc) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxBVH34MidphaseDesc) == 12, "assertion failed:");
        static_assert(sizeof(physx_PxMidphaseDesc) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxBVHDesc) == 40, "assertion failed:");
        static_assert(sizeof(physx_PxCookingParams) == 56, "assertion failed:");
        static_assert(sizeof(physx_PxDefaultMemoryOutputStream) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxDefaultMemoryInputData) == 32, "assertion failed:");
        static_assert(sizeof(physx_PxDefaultFileOutputStream) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxDefaultFileInputData) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxSpring) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxDistanceJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxJacobianRow) == 48, "assertion failed:");
        static_assert(sizeof(physx_PxContactJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxFixedJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxJointLimitParameters) == 20, "assertion failed:");
        static_assert(sizeof(physx_PxJointLinearLimit) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxJointLinearLimitPair) == 28, "assertion failed:");
        static_assert(sizeof(physx_PxJointAngularLimitPair) == 28, "assertion failed:");
        static_assert(sizeof(physx_PxJointLimitCone) == 28, "assertion failed:");
        static_assert(sizeof(physx_PxJointLimitPyramid) == 36, "assertion failed:");
        static_assert(sizeof(physx_PxPrismaticJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxRevoluteJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxSphericalJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxD6JointDrive) == 16, "assertion failed:");
        static_assert(sizeof(physx_PxD6Joint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxGearJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxRackAndPinionJoint) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxGroupsMask) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxRigidActorExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxMassProperties) == 52, "assertion failed:");
        static_assert(sizeof(physx_PxRigidBodyExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxShapeExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxMeshOverlapUtil) == 1040, "assertion failed:");
        static_assert(sizeof(physx_PxXmlMiscParameter) == 20, "assertion failed:");
        static_assert(sizeof(physx_PxSerialization) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxStringTableExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxBroadPhaseExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxSceneQueryExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxSamplingExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxPoissonSampler) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxTriangleMeshPoissonSampler) == 8, "assertion failed:");
        static_assert(sizeof(physx_PxTetrahedronMeshExt) == 1, "assertion failed:");
        static_assert(sizeof(physx_PxRepXObject) == 24, "assertion failed:");
        static_assert(sizeof(physx_PxRepXInstantiationArgs) == 24, "assertion failed:");
