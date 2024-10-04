#include <stdint.h>
#include "physx_generated_enums.h"

typedef struct physx_PxMat34 {
    char _unused [0];
} physx_PxMat34;

typedef struct physx_PxAllocatorCallback physx_PxAllocatorCallback;
struct physx_PxAllocatorCallback {
    void* _vtable;
};

typedef struct physx_PxAssertHandler physx_PxAssertHandler;
struct physx_PxAssertHandler {
    void* _vtable;
};

typedef struct physx_PxFoundation physx_PxFoundation;
struct physx_PxFoundation {
    void* _vtable;
};

typedef struct physx_PxVirtualAllocatorCallback physx_PxVirtualAllocatorCallback;
struct physx_PxVirtualAllocatorCallback {
    void* _vtable;
};

typedef union physx_PxTempAllocatorChunk physx_PxTempAllocatorChunk;
union physx_PxTempAllocatorChunk {
    physx_PxTempAllocatorChunk* mNext;
    uint32_t mIndex;
    uint8_t mPad16;
};

typedef struct physx_PxLogTwo {
    char _unused [0];
} physx_PxLogTwo;

typedef struct physx_PxUnConst {
    char _unused [0];
} physx_PxUnConst;

typedef struct physx_PxErrorCallback physx_PxErrorCallback;
struct physx_PxErrorCallback {
    void* _vtable;
};

typedef struct physx_PxAllocationListener physx_PxAllocationListener;
struct physx_PxAllocationListener {
    void* _vtable;
};

typedef struct physx_PxHash {
    char _unused [0];
} physx_PxHash;

typedef struct physx_PxInputStream physx_PxInputStream;
struct physx_PxInputStream {
    void* _vtable;
};

typedef struct physx_PxInputData physx_PxInputData;
struct physx_PxInputData {
    void* _vtable;
};

typedef struct physx_PxOutputStream physx_PxOutputStream;
struct physx_PxOutputStream {
    void* _vtable;
};

typedef struct physx_PxProfilerCallback physx_PxProfilerCallback;
struct physx_PxProfilerCallback {
    void* _vtable;
};

typedef struct physx_PxRunnable physx_PxRunnable;
struct physx_PxRunnable {
    void* _vtable;
};

typedef struct physx_PxRenderBuffer physx_PxRenderBuffer;
struct physx_PxRenderBuffer {
    void* _vtable;
};

typedef struct physx_PxProcessPxBaseCallback physx_PxProcessPxBaseCallback;
struct physx_PxProcessPxBaseCallback {
    void* _vtable;
};

typedef struct physx_PxSerializationContext physx_PxSerializationContext;
struct physx_PxSerializationContext {
    void* _vtable;
};

typedef struct physx_PxSerializationRegistry physx_PxSerializationRegistry;
struct physx_PxSerializationRegistry {
    void* _vtable;
};

typedef struct physx_PxCollection physx_PxCollection;
struct physx_PxCollection {
    void* _vtable;
};

typedef struct physx_PxTypeInfo {
    char _unused [0];
} physx_PxTypeInfo;

typedef struct physx_PxFEMSoftBodyMaterial {
    char _unused [0];
} physx_PxFEMSoftBodyMaterial;

typedef struct physx_PxFEMClothMaterial {
    char _unused [0];
} physx_PxFEMClothMaterial;

typedef struct physx_PxPBDMaterial {
    char _unused [0];
} physx_PxPBDMaterial;

typedef struct physx_PxFLIPMaterial {
    char _unused [0];
} physx_PxFLIPMaterial;

typedef struct physx_PxMPMMaterial {
    char _unused [0];
} physx_PxMPMMaterial;

typedef struct physx_PxCustomMaterial {
    char _unused [0];
} physx_PxCustomMaterial;

typedef struct physx_PxBVH33TriangleMesh {
    char _unused [0];
} physx_PxBVH33TriangleMesh;

typedef struct physx_PxParticleSystem {
    char _unused [0];
} physx_PxParticleSystem;

typedef struct physx_PxPBDParticleSystem {
    char _unused [0];
} physx_PxPBDParticleSystem;

typedef struct physx_PxFLIPParticleSystem {
    char _unused [0];
} physx_PxFLIPParticleSystem;

typedef struct physx_PxMPMParticleSystem {
    char _unused [0];
} physx_PxMPMParticleSystem;

typedef struct physx_PxCustomParticleSystem {
    char _unused [0];
} physx_PxCustomParticleSystem;

typedef struct physx_PxSoftBody {
    char _unused [0];
} physx_PxSoftBody;

typedef struct physx_PxFEMCloth {
    char _unused [0];
} physx_PxFEMCloth;

typedef struct physx_PxHairSystem {
    char _unused [0];
} physx_PxHairSystem;

typedef struct physx_PxParticleBuffer {
    char _unused [0];
} physx_PxParticleBuffer;

typedef struct physx_PxParticleAndDiffuseBuffer {
    char _unused [0];
} physx_PxParticleAndDiffuseBuffer;

typedef struct physx_PxParticleClothBuffer {
    char _unused [0];
} physx_PxParticleClothBuffer;

typedef struct physx_PxParticleRigidBuffer {
    char _unused [0];
} physx_PxParticleRigidBuffer;

typedef struct physx_PxStringTable physx_PxStringTable;
struct physx_PxStringTable {
    void* _vtable;
};

typedef struct physx_PxSerializer physx_PxSerializer;
struct physx_PxSerializer {
    void* _vtable;
};

typedef struct physx_PxInsertionCallback physx_PxInsertionCallback;
struct physx_PxInsertionCallback {
    void* _vtable;
};

typedef struct physx_PxTaskManager physx_PxTaskManager;
struct physx_PxTaskManager {
    void* _vtable;
};

typedef struct physx_PxCpuDispatcher physx_PxCpuDispatcher;
struct physx_PxCpuDispatcher {
    void* _vtable;
};

typedef struct physx_PxBVHRaycastCallback physx_PxBVHRaycastCallback;
struct physx_PxBVHRaycastCallback {
    void* _vtable;
};

typedef struct physx_PxBVHOverlapCallback physx_PxBVHOverlapCallback;
struct physx_PxBVHOverlapCallback {
    void* _vtable;
};

typedef struct physx_PxBVHTraversalCallback physx_PxBVHTraversalCallback;
struct physx_PxBVHTraversalCallback {
    void* _vtable;
};

typedef struct physx_PxContactBuffer {
    char _unused [0];
} physx_PxContactBuffer;

typedef struct physx_PxRenderOutput {
    char _unused [0];
} physx_PxRenderOutput;

typedef struct physx_PxCustomGeometryCallbacks physx_PxCustomGeometryCallbacks;
struct physx_PxCustomGeometryCallbacks {
    void* _vtable;
};

typedef union physx_Px1DConstraintMods physx_Px1DConstraintMods;
union physx_Px1DConstraintMods {
    physx_PxSpringModifiers spring;
    physx_PxRestitutionModifiers bounce;
};

typedef struct physx_PxConstraintVisualizer physx_PxConstraintVisualizer;
struct physx_PxConstraintVisualizer {
    void* _vtable;
};

typedef struct physx_PxConstraintConnector physx_PxConstraintConnector;
struct physx_PxConstraintConnector {
    void* _vtable;
};

typedef struct physx_PxConstraintAllocator physx_PxConstraintAllocator;
struct physx_PxConstraintAllocator {
    void* _vtable;
};

typedef struct physx_PxContactModifyCallback physx_PxContactModifyCallback;
struct physx_PxContactModifyCallback {
    void* _vtable;
};

typedef struct physx_PxCCDContactModifyCallback physx_PxCCDContactModifyCallback;
struct physx_PxCCDContactModifyCallback {
    void* _vtable;
};

typedef struct physx_PxDeletionListener physx_PxDeletionListener;
struct physx_PxDeletionListener {
    void* _vtable;
};

typedef struct physx_PxSimulationFilterCallback physx_PxSimulationFilterCallback;
struct physx_PxSimulationFilterCallback {
    void* _vtable;
};

typedef struct physx_PxLockedData physx_PxLockedData;
struct physx_PxLockedData {
    void* _vtable;
};

typedef struct physx_PxCudaContextManager {
    char _unused [0];
} physx_PxCudaContextManager;

typedef struct physx_PxParticleRigidAttachment {
    char _unused [0];
} physx_PxParticleRigidAttachment;

typedef struct physx_PxOmniPvd {
    char _unused [0];
} physx_PxOmniPvd;

typedef struct physx_PxPhysics physx_PxPhysics;
struct physx_PxPhysics {
    void* _vtable;
};

typedef struct physx_PxQueryFilterCallback physx_PxQueryFilterCallback;
struct physx_PxQueryFilterCallback {
    void* _vtable;
};

typedef struct physx_PxSceneQuerySystemBase physx_PxSceneQuerySystemBase;
struct physx_PxSceneQuerySystemBase {
    void* _vtable;
};

typedef struct physx_PxSceneSQSystem physx_PxSceneSQSystem;
struct physx_PxSceneSQSystem {
    void* _vtable;
};

typedef struct physx_PxSceneQuerySystem physx_PxSceneQuerySystem;
struct physx_PxSceneQuerySystem {
    void* _vtable;
};

typedef struct physx_PxBroadPhaseRegions physx_PxBroadPhaseRegions;
struct physx_PxBroadPhaseRegions {
    void* _vtable;
};

typedef struct physx_PxBroadPhase physx_PxBroadPhase;
struct physx_PxBroadPhase {
    void* _vtable;
};

typedef struct physx_PxAABBManager physx_PxAABBManager;
struct physx_PxAABBManager {
    void* _vtable;
};

typedef struct physx_PxPvdSceneClient physx_PxPvdSceneClient;
struct physx_PxPvdSceneClient {
    void* _vtable;
};

typedef struct physx_PxBroadPhaseCallback physx_PxBroadPhaseCallback;
struct physx_PxBroadPhaseCallback {
    void* _vtable;
};

typedef struct physx_PxSimulationEventCallback physx_PxSimulationEventCallback;
struct physx_PxSimulationEventCallback {
    void* _vtable;
};

typedef struct physx_PxObstacleContext physx_PxObstacleContext;
struct physx_PxObstacleContext {
    void* _vtable;
};

typedef struct physx_PxUserControllerHitReport physx_PxUserControllerHitReport;
struct physx_PxUserControllerHitReport {
    void* _vtable;
};

typedef struct physx_PxControllerFilterCallback physx_PxControllerFilterCallback;
struct physx_PxControllerFilterCallback {
    void* _vtable;
};

typedef struct physx_PxController physx_PxController;
struct physx_PxController {
    void* _vtable;
};

typedef struct physx_PxBoxController physx_PxBoxController;
struct physx_PxBoxController {
    void* _vtable;
};

typedef struct physx_PxCapsuleController physx_PxCapsuleController;
struct physx_PxCapsuleController {
    void* _vtable;
};

typedef struct physx_PxControllerBehaviorCallback physx_PxControllerBehaviorCallback;
struct physx_PxControllerBehaviorCallback {
    void* _vtable;
};

typedef struct physx_PxControllerManager physx_PxControllerManager;
struct physx_PxControllerManager {
    void* _vtable;
};

typedef struct physx_PxDefaultAllocator physx_PxDefaultAllocator;
struct physx_PxDefaultAllocator {
    void* _vtable;
};

typedef struct physx_PxDefaultErrorCallback physx_PxDefaultErrorCallback;
struct physx_PxDefaultErrorCallback {
    void* _vtable;
};

typedef struct physx_PxBinaryConverter {
    char _unused [0];
} physx_PxBinaryConverter;

typedef struct physx_PxDefaultCpuDispatcher physx_PxDefaultCpuDispatcher;
struct physx_PxDefaultCpuDispatcher {
    void* _vtable;
};

typedef struct physx_PxBatchQueryExt physx_PxBatchQueryExt;
struct physx_PxBatchQueryExt {
    void* _vtable;
};

typedef struct physx_PxCustomSceneQuerySystem physx_PxCustomSceneQuerySystem;
struct physx_PxCustomSceneQuerySystem {
    void* _vtable;
};

typedef struct physx_PxCustomSceneQuerySystemAdapter physx_PxCustomSceneQuerySystemAdapter;
struct physx_PxCustomSceneQuerySystemAdapter {
    void* _vtable;
};

typedef struct physx_PxCooking {
    char _unused [0];
} physx_PxCooking;

typedef struct physx_XmlMemoryAllocator {
    char _unused [0];
} physx_XmlMemoryAllocator;

typedef struct physx_XmlWriter {
    char _unused [0];
} physx_XmlWriter;

typedef struct physx_XmlReader {
    char _unused [0];
} physx_XmlReader;

typedef struct physx_MemoryBuffer {
    char _unused [0];
} physx_MemoryBuffer;

typedef struct physx_PxRepXSerializer physx_PxRepXSerializer;
struct physx_PxRepXSerializer {
    void* _vtable;
};

typedef struct physx_PxVehicleWheels4SimData {
    char _unused [0];
} physx_PxVehicleWheels4SimData;

typedef struct physx_PxVehicleWheels4DynData {
    char _unused [0];
} physx_PxVehicleWheels4DynData;

typedef struct physx_PxVehicleTireForceCalculator {
    char _unused [0];
} physx_PxVehicleTireForceCalculator;

typedef struct physx_PxVehicleDrivableSurfaceToTireFrictionPairs {
    char _unused [0];
} physx_PxVehicleDrivableSurfaceToTireFrictionPairs;

typedef struct physx_PxVehicleTelemetryData {
    char _unused [0];
} physx_PxVehicleTelemetryData;

typedef struct physx_PxPvd physx_PxPvd;
struct physx_PxPvd {
    void* _vtable;
};

typedef struct physx_PxPvdTransport physx_PxPvdTransport;
struct physx_PxPvdTransport {
    void* _vtable;
};
void      physx_PxAllocatorCallback_delete(physx_PxAllocatorCallback* self_);

    /// Allocates size bytes of memory, which must be 16-byte aligned.
    ///
    /// This method should never return NULL.  If you run out of memory, then
    /// you should terminate the app or take some other appropriate action.
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    ///
    /// The allocated block of memory.
void*      physx_PxAllocatorCallback_allocate_mut(physx_PxAllocatorCallback* self_, size_t size, char const* typeName, char const* filename, int32_t line);

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
void      physx_PxAllocatorCallback_deallocate_mut(physx_PxAllocatorCallback* self_, void* ptr);

void      physx_PxAssertHandler_delete(physx_PxAssertHandler* self_);

physx_PxAssertHandler*      physx_phys_PxGetAssertHandler();

void      physx_phys_PxSetAssertHandler(physx_PxAssertHandler* handler);

    /// Destroys the instance it is called on.
    ///
    /// The operation will fail, if there are still modules referencing the foundation object. Release all dependent modules
    /// prior to calling this method.
void      physx_PxFoundation_release_mut(physx_PxFoundation* self_);

    /// retrieves error callback
physx_PxErrorCallback*      physx_PxFoundation_getErrorCallback_mut(physx_PxFoundation* self_);

    /// Sets mask of errors to report.
void      physx_PxFoundation_setErrorLevel_mut(physx_PxFoundation* self_, uint32_t mask);

    /// Retrieves mask of errors to be reported.
uint32_t      physx_PxFoundation_getErrorLevel(physx_PxFoundation const* self_);

    /// Retrieves the allocator this object was created with.
physx_PxAllocatorCallback*      physx_PxFoundation_getAllocatorCallback_mut(physx_PxFoundation* self_);

    /// Retrieves if allocation names are being passed to allocator callback.
bool      physx_PxFoundation_getReportAllocationNames(physx_PxFoundation const* self_);

    /// Set if allocation names are being passed to allocator callback.
    ///
    /// Enabled by default in debug and checked build, disabled by default in profile and release build.
void      physx_PxFoundation_setReportAllocationNames_mut(physx_PxFoundation* self_, bool value);

void      physx_PxFoundation_registerAllocationListener_mut(physx_PxFoundation* self_, physx_PxAllocationListener* listener);

void      physx_PxFoundation_deregisterAllocationListener_mut(physx_PxFoundation* self_, physx_PxAllocationListener* listener);

void      physx_PxFoundation_registerErrorCallback_mut(physx_PxFoundation* self_, physx_PxErrorCallback* callback);

void      physx_PxFoundation_deregisterErrorCallback_mut(physx_PxFoundation* self_, physx_PxErrorCallback* callback);

    /// Creates an instance of the foundation class
    ///
    /// The foundation class is needed to initialize higher level SDKs. There may be only one instance per process.
    /// Calling this method after an instance has been created already will result in an error message and NULL will be
    /// returned.
    ///
    /// Foundation instance on success, NULL if operation failed
physx_PxFoundation*      physx_phys_PxCreateFoundation(uint32_t version, physx_PxAllocatorCallback* allocator, physx_PxErrorCallback* errorCallback);

void      physx_phys_PxSetFoundationInstance(physx_PxFoundation* foundation);

physx_PxFoundation*      physx_phys_PxGetFoundation();

    /// Get the callback that will be used for all profiling.
physx_PxProfilerCallback*      physx_phys_PxGetProfilerCallback();

    /// Set the callback that will be used for all profiling.
void      physx_phys_PxSetProfilerCallback(physx_PxProfilerCallback* profiler);

    /// Get the allocator callback
physx_PxAllocatorCallback*      physx_phys_PxGetAllocatorCallback();

    /// Get the broadcasting allocator callback
physx_PxAllocatorCallback*      physx_phys_PxGetBroadcastAllocator();

    /// Get the error callback
physx_PxErrorCallback*      physx_phys_PxGetErrorCallback();

    /// Get the broadcasting error callback
physx_PxErrorCallback*      physx_phys_PxGetBroadcastError();

    /// Get the warn once timestamp
uint32_t      physx_phys_PxGetWarnOnceTimeStamp();

    /// Decrement the ref count of PxFoundation
void      physx_phys_PxDecFoundationRefCount();

    /// Increment the ref count of PxFoundation
void      physx_phys_PxIncFoundationRefCount();

physx_PxAllocator      physx_PxAllocator_new(char const* anon_param0);

void*      physx_PxAllocator_allocate_mut(physx_PxAllocator* self_, size_t size, char const* file, int32_t line);

void      physx_PxAllocator_deallocate_mut(physx_PxAllocator* self_, void* ptr);

physx_PxRawAllocator      physx_PxRawAllocator_new(char const* anon_param0);

void*      physx_PxRawAllocator_allocate_mut(physx_PxRawAllocator* self_, size_t size, char const* anon_param1, int32_t anon_param2);

void      physx_PxRawAllocator_deallocate_mut(physx_PxRawAllocator* self_, void* ptr);

void      physx_PxVirtualAllocatorCallback_delete(physx_PxVirtualAllocatorCallback* self_);

void*      physx_PxVirtualAllocatorCallback_allocate_mut(physx_PxVirtualAllocatorCallback* self_, size_t size, int32_t group, char const* file, int32_t line);

void      physx_PxVirtualAllocatorCallback_deallocate_mut(physx_PxVirtualAllocatorCallback* self_, void* ptr);

physx_PxVirtualAllocator      physx_PxVirtualAllocator_new(physx_PxVirtualAllocatorCallback* callback, int32_t group);

void*      physx_PxVirtualAllocator_allocate_mut(physx_PxVirtualAllocator* self_, size_t size, char const* file, int32_t line);

void      physx_PxVirtualAllocator_deallocate_mut(physx_PxVirtualAllocator* self_, void* ptr);

physx_PxTempAllocatorChunk      physx_PxTempAllocatorChunk_new();

physx_PxTempAllocator      physx_PxTempAllocator_new(char const* anon_param0);

void*      physx_PxTempAllocator_allocate_mut(physx_PxTempAllocator* self_, size_t size, char const* file, int32_t line);

void      physx_PxTempAllocator_deallocate_mut(physx_PxTempAllocator* self_, void* ptr);

    /// Sets the bytes of the provided buffer to zero.
    ///
    /// Pointer to memory block (same as input)
void*      physx_phys_PxMemZero(void* dest, uint32_t count);

    /// Sets the bytes of the provided buffer to the specified value.
    ///
    /// Pointer to memory block (same as input)
void*      physx_phys_PxMemSet(void* dest, int32_t c, uint32_t count);

    /// Copies the bytes of one memory block to another. The memory blocks must not overlap.
    ///
    /// Use [`PxMemMove`] if memory blocks overlap.
    ///
    /// Pointer to destination memory block
void*      physx_phys_PxMemCopy(void* dest, void const* src, uint32_t count);

    /// Copies the bytes of one memory block to another. The memory blocks can overlap.
    ///
    /// Use [`PxMemCopy`] if memory blocks do not overlap.
    ///
    /// Pointer to destination memory block
void*      physx_phys_PxMemMove(void* dest, void const* src, uint32_t count);

    /// Mark a specified amount of memory with 0xcd pattern. This is used to check that the meta data
    /// definition for serialized classes is complete in checked builds.
void      physx_phys_PxMarkSerializedMemory(void* ptr, uint32_t byteSize);

void      physx_phys_PxMemoryBarrier();

    /// Return the index of the highest set bit. Undefined for zero arg.
uint32_t      physx_phys_PxHighestSetBitUnsafe(uint32_t v);

    /// Return the index of the highest set bit. Undefined for zero arg.
uint32_t      physx_phys_PxLowestSetBitUnsafe(uint32_t v);

    /// Returns the index of the highest set bit. Returns 32 for v=0.
uint32_t      physx_phys_PxCountLeadingZeros(uint32_t v);

    /// Prefetch aligned 64B x86, 32b ARM around
void      physx_phys_PxPrefetchLine(void const* ptr, uint32_t offset);

    /// Prefetch
    /// bytes starting at
void      physx_phys_PxPrefetch(void const* ptr, uint32_t count);

uint32_t      physx_phys_PxBitCount(uint32_t v);

bool      physx_phys_PxIsPowerOfTwo(uint32_t x);

uint32_t      physx_phys_PxNextPowerOfTwo(uint32_t x);

    /// Return the index of the highest set bit. Not valid for zero arg.
uint32_t      physx_phys_PxLowestSetBit(uint32_t x);

    /// Return the index of the highest set bit. Not valid for zero arg.
uint32_t      physx_phys_PxHighestSetBit(uint32_t x);

uint32_t      physx_phys_PxILog2(uint32_t num);

    /// default constructor leaves data uninitialized.
physx_PxVec3      physx_PxVec3_new();

    /// zero constructor.
physx_PxVec3      physx_PxVec3_new_1(int32_t anon_param0);

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
physx_PxVec3      physx_PxVec3_new_2(float a);

    /// Initializes from 3 scalar parameters.
physx_PxVec3      physx_PxVec3_new_3(float nx, float ny, float nz);

    /// tests for exact zero vector
bool      physx_PxVec3_isZero(physx_PxVec3 const* self_);

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
bool      physx_PxVec3_isFinite(physx_PxVec3 const* self_);

    /// is normalized - used by API parameter validation
bool      physx_PxVec3_isNormalized(physx_PxVec3 const* self_);

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
float      physx_PxVec3_magnitudeSquared(physx_PxVec3 const* self_);

    /// returns the magnitude
float      physx_PxVec3_magnitude(physx_PxVec3 const* self_);

    /// returns the scalar product of this and other.
float      physx_PxVec3_dot(physx_PxVec3 const* self_, physx_PxVec3 const* v);

    /// cross product
physx_PxVec3      physx_PxVec3_cross(physx_PxVec3 const* self_, physx_PxVec3 const* v);

    /// returns a unit vector
physx_PxVec3      physx_PxVec3_getNormalized(physx_PxVec3 const* self_);

    /// normalizes the vector in place
float      physx_PxVec3_normalize_mut(physx_PxVec3* self_);

    /// normalizes the vector in place. Does nothing if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// Returns vector magnitude if >= PX_NORMALIZATION_EPSILON and 0.0f otherwise.
float      physx_PxVec3_normalizeSafe_mut(physx_PxVec3* self_);

    /// normalizes the vector in place. Asserts if vector magnitude is under PX_NORMALIZATION_EPSILON.
    /// returns vector magnitude.
float      physx_PxVec3_normalizeFast_mut(physx_PxVec3* self_);

    /// a[i] * b[i], for all i.
physx_PxVec3      physx_PxVec3_multiply(physx_PxVec3 const* self_, physx_PxVec3 const* a);

    /// element-wise minimum
physx_PxVec3      physx_PxVec3_minimum(physx_PxVec3 const* self_, physx_PxVec3 const* v);

    /// returns MIN(x, y, z);
float      physx_PxVec3_minElement(physx_PxVec3 const* self_);

    /// element-wise maximum
physx_PxVec3      physx_PxVec3_maximum(physx_PxVec3 const* self_, physx_PxVec3 const* v);

    /// returns MAX(x, y, z);
float      physx_PxVec3_maxElement(physx_PxVec3 const* self_);

    /// returns absolute values of components;
physx_PxVec3      physx_PxVec3_abs(physx_PxVec3 const* self_);

physx_PxVec3Padded*      physx_PxVec3Padded_new_alloc();

void      physx_PxVec3Padded_delete(physx_PxVec3Padded* self_);

physx_PxVec3Padded*      physx_PxVec3Padded_new_alloc_1(physx_PxVec3 const* p);

physx_PxVec3Padded*      physx_PxVec3Padded_new_alloc_2(float f);

    /// Default constructor, does not do any initialization.
physx_PxQuat      physx_PxQuat_new();

    /// identity constructor
physx_PxQuat      physx_PxQuat_new_1(int32_t anon_param0);

    /// Constructor from a scalar: sets the real part w to the scalar value, and the imaginary parts (x,y,z) to zero
physx_PxQuat      physx_PxQuat_new_2(float r);

    /// Constructor. Take note of the order of the elements!
physx_PxQuat      physx_PxQuat_new_3(float nx, float ny, float nz, float nw);

    /// Creates from angle-axis representation.
    ///
    /// Axis must be normalized!
    ///
    /// Angle is in radians!
    ///
    /// Unit:
    /// Radians
physx_PxQuat      physx_PxQuat_new_4(float angleRadians, physx_PxVec3 const* unitAxis);

    /// Creates from orientation matrix.
physx_PxQuat      physx_PxQuat_new_5(physx_PxMat33 const* m);

    /// returns true if quat is identity
bool      physx_PxQuat_isIdentity(physx_PxQuat const* self_);

    /// returns true if all elements are finite (not NAN or INF, etc.)
bool      physx_PxQuat_isFinite(physx_PxQuat const* self_);

    /// returns true if finite and magnitude is close to unit
bool      physx_PxQuat_isUnit(physx_PxQuat const* self_);

    /// returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs
    /// isValid
bool      physx_PxQuat_isSane(physx_PxQuat const* self_);

    /// converts this quaternion to angle-axis representation
void      physx_PxQuat_toRadiansAndUnitAxis(physx_PxQuat const* self_, float* angle, physx_PxVec3* axis);

    /// Gets the angle between this quat and the identity quaternion.
    ///
    /// Unit:
    /// Radians
float      physx_PxQuat_getAngle(physx_PxQuat const* self_);

    /// Gets the angle between this quat and the argument
    ///
    /// Unit:
    /// Radians
float      physx_PxQuat_getAngle_1(physx_PxQuat const* self_, physx_PxQuat const* q);

    /// This is the squared 4D vector length, should be 1 for unit quaternions.
float      physx_PxQuat_magnitudeSquared(physx_PxQuat const* self_);

    /// returns the scalar product of this and other.
float      physx_PxQuat_dot(physx_PxQuat const* self_, physx_PxQuat const* v);

physx_PxQuat      physx_PxQuat_getNormalized(physx_PxQuat const* self_);

float      physx_PxQuat_magnitude(physx_PxQuat const* self_);

    /// maps to the closest unit quaternion.
float      physx_PxQuat_normalize_mut(physx_PxQuat* self_);

physx_PxQuat      physx_PxQuat_getConjugate(physx_PxQuat const* self_);

physx_PxVec3      physx_PxQuat_getImaginaryPart(physx_PxQuat const* self_);

    /// brief computes rotation of x-axis
physx_PxVec3      physx_PxQuat_getBasisVector0(physx_PxQuat const* self_);

    /// brief computes rotation of y-axis
physx_PxVec3      physx_PxQuat_getBasisVector1(physx_PxQuat const* self_);

    /// brief computes rotation of z-axis
physx_PxVec3      physx_PxQuat_getBasisVector2(physx_PxQuat const* self_);

    /// rotates passed vec by this (assumed unitary)
physx_PxVec3      physx_PxQuat_rotate(physx_PxQuat const* self_, physx_PxVec3 const* v);

    /// inverse rotates passed vec by this (assumed unitary)
physx_PxVec3      physx_PxQuat_rotateInv(physx_PxQuat const* self_, physx_PxVec3 const* v);

physx_PxTransform      physx_PxTransform_new();

physx_PxTransform      physx_PxTransform_new_1(physx_PxVec3 const* position);

physx_PxTransform      physx_PxTransform_new_2(int32_t anon_param0);

physx_PxTransform      physx_PxTransform_new_3(physx_PxQuat const* orientation);

physx_PxTransform      physx_PxTransform_new_4(float x, float y, float z, physx_PxQuat aQ);

physx_PxTransform      physx_PxTransform_new_5(physx_PxVec3 const* p0, physx_PxQuat const* q0);

physx_PxTransform      physx_PxTransform_new_6(physx_PxMat44 const* m);

physx_PxTransform      physx_PxTransform_getInverse(physx_PxTransform const* self_);

physx_PxVec3      physx_PxTransform_transform(physx_PxTransform const* self_, physx_PxVec3 const* input);

physx_PxVec3      physx_PxTransform_transformInv(physx_PxTransform const* self_, physx_PxVec3 const* input);

physx_PxVec3      physx_PxTransform_rotate(physx_PxTransform const* self_, physx_PxVec3 const* input);

physx_PxVec3      physx_PxTransform_rotateInv(physx_PxTransform const* self_, physx_PxVec3 const* input);

    /// Transform transform to parent (returns compound transform: first src, then *this)
physx_PxTransform      physx_PxTransform_transform_1(physx_PxTransform const* self_, physx_PxTransform const* src);

    /// returns true if finite and q is a unit quaternion
bool      physx_PxTransform_isValid(physx_PxTransform const* self_);

    /// returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error
    /// vs isValid
bool      physx_PxTransform_isSane(physx_PxTransform const* self_);

    /// returns true if all elems are finite (not NAN or INF, etc.)
bool      physx_PxTransform_isFinite(physx_PxTransform const* self_);

    /// Transform transform from parent (returns compound transform: first src, then this->inverse)
physx_PxTransform      physx_PxTransform_transformInv_1(physx_PxTransform const* self_, physx_PxTransform const* src);

    /// return a normalized transform (i.e. one in which the quaternion has unit magnitude)
physx_PxTransform      physx_PxTransform_getNormalized(physx_PxTransform const* self_);

    /// Default constructor
physx_PxMat33      physx_PxMat33_new();

    /// identity constructor
physx_PxMat33      physx_PxMat33_new_1(int32_t anon_param0);

    /// zero constructor
physx_PxMat33      physx_PxMat33_new_2(int32_t anon_param0);

    /// Construct from three base vectors
physx_PxMat33      physx_PxMat33_new_3(physx_PxVec3 const* col0, physx_PxVec3 const* col1, physx_PxVec3 const* col2);

    /// constructor from a scalar, which generates a multiple of the identity matrix
physx_PxMat33      physx_PxMat33_new_4(float r);

    /// Construct from float[9]
physx_PxMat33      physx_PxMat33_new_5(float* values);

    /// Construct from a quaternion
physx_PxMat33      physx_PxMat33_new_6(physx_PxQuat const* q);

    /// Construct from diagonal, off-diagonals are zero.
physx_PxMat33      physx_PxMat33_createDiagonal(physx_PxVec3 const* d);

    /// Computes the outer product of two vectors
physx_PxMat33      physx_PxMat33_outer(physx_PxVec3 const* a, physx_PxVec3 const* b);

    /// Get transposed matrix
physx_PxMat33      physx_PxMat33_getTranspose(physx_PxMat33 const* self_);

    /// Get the real inverse
physx_PxMat33      physx_PxMat33_getInverse(physx_PxMat33 const* self_);

    /// Get determinant
float      physx_PxMat33_getDeterminant(physx_PxMat33 const* self_);

    /// Transform vector by matrix, equal to v' = M*v
physx_PxVec3      physx_PxMat33_transform(physx_PxMat33 const* self_, physx_PxVec3 const* other);

    /// Transform vector by matrix transpose, v' = M^t*v
physx_PxVec3      physx_PxMat33_transformTranspose(physx_PxMat33 const* self_, physx_PxVec3 const* other);

float const*      physx_PxMat33_front(physx_PxMat33 const* self_);

    /// Default constructor, not performing any initialization for performance reason.
    ///
    /// Use empty() function below to construct empty bounds.
physx_PxBounds3      physx_PxBounds3_new();

    /// Construct from two bounding points
physx_PxBounds3      physx_PxBounds3_new_1(physx_PxVec3 const* minimum, physx_PxVec3 const* maximum);

    /// Return empty bounds.
physx_PxBounds3      physx_PxBounds3_empty();

    /// returns the AABB containing v0 and v1.
physx_PxBounds3      physx_PxBounds3_boundsOfPoints(physx_PxVec3 const* v0, physx_PxVec3 const* v1);

    /// returns the AABB from center and extents vectors.
physx_PxBounds3      physx_PxBounds3_centerExtents(physx_PxVec3 const* center, physx_PxVec3 const* extent);

    /// Construct from center, extent, and (not necessarily orthogonal) basis
physx_PxBounds3      physx_PxBounds3_basisExtent(physx_PxVec3 const* center, physx_PxMat33 const* basis, physx_PxVec3 const* extent);

    /// Construct from pose and extent
physx_PxBounds3      physx_PxBounds3_poseExtent(physx_PxTransform const* pose, physx_PxVec3 const* extent);

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
physx_PxBounds3      physx_PxBounds3_transformSafe(physx_PxMat33 const* matrix, physx_PxBounds3 const* bounds);

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
physx_PxBounds3      physx_PxBounds3_transformFast(physx_PxMat33 const* matrix, physx_PxBounds3 const* bounds);

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// This version is safe to call for empty bounds.
physx_PxBounds3      physx_PxBounds3_transformSafe_1(physx_PxTransform const* transform, physx_PxBounds3 const* bounds);

    /// gets the transformed bounds of the passed AABB (resulting in a bigger AABB).
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`transformSafe`]() instead.
physx_PxBounds3      physx_PxBounds3_transformFast_1(physx_PxTransform const* transform, physx_PxBounds3 const* bounds);

    /// Sets empty to true
void      physx_PxBounds3_setEmpty_mut(physx_PxBounds3* self_);

    /// Sets the bounds to maximum size [-PX_MAX_BOUNDS_EXTENTS, PX_MAX_BOUNDS_EXTENTS].
void      physx_PxBounds3_setMaximal_mut(physx_PxBounds3* self_);

    /// expands the volume to include v
void      physx_PxBounds3_include_mut(physx_PxBounds3* self_, physx_PxVec3 const* v);

    /// expands the volume to include b.
void      physx_PxBounds3_include_mut_1(physx_PxBounds3* self_, physx_PxBounds3 const* b);

bool      physx_PxBounds3_isEmpty(physx_PxBounds3 const* self_);

    /// indicates whether the intersection of this and b is empty or not.
bool      physx_PxBounds3_intersects(physx_PxBounds3 const* self_, physx_PxBounds3 const* b);

    /// computes the 1D-intersection between two AABBs, on a given axis.
bool      physx_PxBounds3_intersects1D(physx_PxBounds3 const* self_, physx_PxBounds3 const* a, uint32_t axis);

    /// indicates if these bounds contain v.
bool      physx_PxBounds3_contains(physx_PxBounds3 const* self_, physx_PxVec3 const* v);

    /// checks a box is inside another box.
bool      physx_PxBounds3_isInside(physx_PxBounds3 const* self_, physx_PxBounds3 const* box_);

    /// returns the center of this axis aligned box.
physx_PxVec3      physx_PxBounds3_getCenter(physx_PxBounds3 const* self_);

    /// get component of the box's center along a given axis
float      physx_PxBounds3_getCenter_1(physx_PxBounds3 const* self_, uint32_t axis);

    /// get component of the box's extents along a given axis
float      physx_PxBounds3_getExtents(physx_PxBounds3 const* self_, uint32_t axis);

    /// returns the dimensions (width/height/depth) of this axis aligned box.
physx_PxVec3      physx_PxBounds3_getDimensions(physx_PxBounds3 const* self_);

    /// returns the extents, which are half of the width/height/depth.
physx_PxVec3      physx_PxBounds3_getExtents_1(physx_PxBounds3 const* self_);

    /// scales the AABB.
    ///
    /// This version is safe to call for empty bounds.
void      physx_PxBounds3_scaleSafe_mut(physx_PxBounds3* self_, float scale);

    /// scales the AABB.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`scaleSafe`]() instead.
void      physx_PxBounds3_scaleFast_mut(physx_PxBounds3* self_, float scale);

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// This version is safe to call for empty bounds.
void      physx_PxBounds3_fattenSafe_mut(physx_PxBounds3* self_, float distance);

    /// fattens the AABB in all 3 dimensions by the given distance.
    ///
    /// Calling this method for empty bounds leads to undefined behavior. Use [`fattenSafe`]() instead.
void      physx_PxBounds3_fattenFast_mut(physx_PxBounds3* self_, float distance);

    /// checks that the AABB values are not NaN
bool      physx_PxBounds3_isFinite(physx_PxBounds3 const* self_);

    /// checks that the AABB values describe a valid configuration.
bool      physx_PxBounds3_isValid(physx_PxBounds3 const* self_);

    /// Finds the closest point in the box to the point p. If p is contained, this will be p, otherwise it
    /// will be the closest point on the surface of the box.
physx_PxVec3      physx_PxBounds3_closestPoint(physx_PxBounds3 const* self_, physx_PxVec3 const* p);

void      physx_PxErrorCallback_delete(physx_PxErrorCallback* self_);

    /// Reports an error code.
void      physx_PxErrorCallback_reportError_mut(physx_PxErrorCallback* self_, int32_t code, char const* message, char const* file, int32_t line);

    /// callback when memory is allocated.
void      physx_PxAllocationListener_onAllocation_mut(physx_PxAllocationListener* self_, size_t size, char const* typeName, char const* filename, int32_t line, void* allocatedMemory);

    /// callback when memory is deallocated.
void      physx_PxAllocationListener_onDeallocation_mut(physx_PxAllocationListener* self_, void* allocatedMemory);

    /// The default constructor.
physx_PxBroadcastingAllocator*      physx_PxBroadcastingAllocator_new_alloc(physx_PxAllocatorCallback* allocator, physx_PxErrorCallback* error);

    /// The default constructor.
void      physx_PxBroadcastingAllocator_delete(physx_PxBroadcastingAllocator* self_);

    /// Allocates size bytes of memory, which must be 16-byte aligned.
    ///
    /// This method should never return NULL.  If you run out of memory, then
    /// you should terminate the app or take some other appropriate action.
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
    ///
    /// The allocated block of memory.
void*      physx_PxBroadcastingAllocator_allocate_mut(physx_PxBroadcastingAllocator* self_, size_t size, char const* typeName, char const* filename, int32_t line);

    /// Frees memory previously allocated by allocate().
    ///
    /// Threading:
    /// This function should be thread safe as it can be called in the context of the user thread
    /// and physics processing thread(s).
void      physx_PxBroadcastingAllocator_deallocate_mut(physx_PxBroadcastingAllocator* self_, void* ptr);

    /// The default constructor.
physx_PxBroadcastingErrorCallback*      physx_PxBroadcastingErrorCallback_new_alloc(physx_PxErrorCallback* errorCallback);

    /// The default destructor.
void      physx_PxBroadcastingErrorCallback_delete(physx_PxBroadcastingErrorCallback* self_);

    /// Reports an error code.
void      physx_PxBroadcastingErrorCallback_reportError_mut(physx_PxBroadcastingErrorCallback* self_, int32_t code, char const* message, char const* file, int32_t line);

    /// Enables floating point exceptions for the scalar and SIMD unit
void      physx_phys_PxEnableFPExceptions();

    /// Disables floating point exceptions for the scalar and SIMD unit
void      physx_phys_PxDisableFPExceptions();

    /// read from the stream. The number of bytes read may be less than the number requested.
    ///
    /// the number of bytes read from the stream.
uint32_t      physx_PxInputStream_read_mut(physx_PxInputStream* self_, void* dest, uint32_t count);

void      physx_PxInputStream_delete(physx_PxInputStream* self_);

    /// return the length of the input data
    ///
    /// size in bytes of the input data
uint32_t      physx_PxInputData_getLength(physx_PxInputData const* self_);

    /// seek to the given offset from the start of the data.
void      physx_PxInputData_seek_mut(physx_PxInputData* self_, uint32_t offset);

    /// return the current offset from the start of the data
    ///
    /// the offset to seek to.
uint32_t      physx_PxInputData_tell(physx_PxInputData const* self_);

void      physx_PxInputData_delete(physx_PxInputData* self_);

    /// write to the stream. The number of bytes written may be less than the number sent.
    ///
    /// the number of bytes written to the stream by this call.
uint32_t      physx_PxOutputStream_write_mut(physx_PxOutputStream* self_, void const* src, uint32_t count);

void      physx_PxOutputStream_delete(physx_PxOutputStream* self_);

    /// default constructor leaves data uninitialized.
physx_PxVec4      physx_PxVec4_new();

    /// zero constructor.
physx_PxVec4      physx_PxVec4_new_1(int32_t anon_param0);

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
physx_PxVec4      physx_PxVec4_new_2(float a);

    /// Initializes from 3 scalar parameters.
physx_PxVec4      physx_PxVec4_new_3(float nx, float ny, float nz, float nw);

    /// Initializes from 3 scalar parameters.
physx_PxVec4      physx_PxVec4_new_4(physx_PxVec3 const* v, float nw);

    /// Initializes from an array of scalar parameters.
physx_PxVec4      physx_PxVec4_new_5(float const* v);

    /// tests for exact zero vector
bool      physx_PxVec4_isZero(physx_PxVec4 const* self_);

    /// returns true if all 3 elems of the vector are finite (not NAN or INF, etc.)
bool      physx_PxVec4_isFinite(physx_PxVec4 const* self_);

    /// is normalized - used by API parameter validation
bool      physx_PxVec4_isNormalized(physx_PxVec4 const* self_);

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
float      physx_PxVec4_magnitudeSquared(physx_PxVec4 const* self_);

    /// returns the magnitude
float      physx_PxVec4_magnitude(physx_PxVec4 const* self_);

    /// returns the scalar product of this and other.
float      physx_PxVec4_dot(physx_PxVec4 const* self_, physx_PxVec4 const* v);

    /// returns a unit vector
physx_PxVec4      physx_PxVec4_getNormalized(physx_PxVec4 const* self_);

    /// normalizes the vector in place
float      physx_PxVec4_normalize_mut(physx_PxVec4* self_);

    /// a[i] * b[i], for all i.
physx_PxVec4      physx_PxVec4_multiply(physx_PxVec4 const* self_, physx_PxVec4 const* a);

    /// element-wise minimum
physx_PxVec4      physx_PxVec4_minimum(physx_PxVec4 const* self_, physx_PxVec4 const* v);

    /// element-wise maximum
physx_PxVec4      physx_PxVec4_maximum(physx_PxVec4 const* self_, physx_PxVec4 const* v);

physx_PxVec3      physx_PxVec4_getXYZ(physx_PxVec4 const* self_);

    /// Default constructor
physx_PxMat44      physx_PxMat44_new();

    /// identity constructor
physx_PxMat44      physx_PxMat44_new_1(int32_t anon_param0);

    /// zero constructor
physx_PxMat44      physx_PxMat44_new_2(int32_t anon_param0);

    /// Construct from four 4-vectors
physx_PxMat44      physx_PxMat44_new_3(physx_PxVec4 const* col0, physx_PxVec4 const* col1, physx_PxVec4 const* col2, physx_PxVec4 const* col3);

    /// constructor that generates a multiple of the identity matrix
physx_PxMat44      physx_PxMat44_new_4(float r);

    /// Construct from three base vectors and a translation
physx_PxMat44      physx_PxMat44_new_5(physx_PxVec3 const* col0, physx_PxVec3 const* col1, physx_PxVec3 const* col2, physx_PxVec3 const* col3);

    /// Construct from float[16]
physx_PxMat44      physx_PxMat44_new_6(float* values);

    /// Construct from a quaternion
physx_PxMat44      physx_PxMat44_new_7(physx_PxQuat const* q);

    /// Construct from a diagonal vector
physx_PxMat44      physx_PxMat44_new_8(physx_PxVec4 const* diagonal);

    /// Construct from Mat33 and a translation
physx_PxMat44      physx_PxMat44_new_9(physx_PxMat33 const* axes, physx_PxVec3 const* position);

physx_PxMat44      physx_PxMat44_new_10(physx_PxTransform const* t);

    /// Get transposed matrix
physx_PxMat44      physx_PxMat44_getTranspose(physx_PxMat44 const* self_);

    /// Transform vector by matrix, equal to v' = M*v
physx_PxVec4      physx_PxMat44_transform(physx_PxMat44 const* self_, physx_PxVec4 const* other);

    /// Transform vector by matrix, equal to v' = M*v
physx_PxVec3      physx_PxMat44_transform_1(physx_PxMat44 const* self_, physx_PxVec3 const* other);

    /// Rotate vector by matrix, equal to v' = M*v
physx_PxVec4      physx_PxMat44_rotate(physx_PxMat44 const* self_, physx_PxVec4 const* other);

    /// Rotate vector by matrix, equal to v' = M*v
physx_PxVec3      physx_PxMat44_rotate_1(physx_PxMat44 const* self_, physx_PxVec3 const* other);

physx_PxVec3      physx_PxMat44_getBasis(physx_PxMat44 const* self_, uint32_t num);

physx_PxVec3      physx_PxMat44_getPosition(physx_PxMat44 const* self_);

void      physx_PxMat44_setPosition_mut(physx_PxMat44* self_, physx_PxVec3 const* position);

float const*      physx_PxMat44_front(physx_PxMat44 const* self_);

void      physx_PxMat44_scale_mut(physx_PxMat44* self_, physx_PxVec4 const* p);

physx_PxMat44      physx_PxMat44_inverseRT(physx_PxMat44 const* self_);

bool      physx_PxMat44_isFinite(physx_PxMat44 const* self_);

    /// Constructor
physx_PxPlane      physx_PxPlane_new();

    /// Constructor from a normal and a distance
physx_PxPlane      physx_PxPlane_new_1(float nx, float ny, float nz, float distance);

    /// Constructor from a normal and a distance
physx_PxPlane      physx_PxPlane_new_2(physx_PxVec3 const* normal, float distance);

    /// Constructor from a point on the plane and a normal
physx_PxPlane      physx_PxPlane_new_3(physx_PxVec3 const* point, physx_PxVec3 const* normal);

    /// Constructor from three points
physx_PxPlane      physx_PxPlane_new_4(physx_PxVec3 const* p0, physx_PxVec3 const* p1, physx_PxVec3 const* p2);

float      physx_PxPlane_distance(physx_PxPlane const* self_, physx_PxVec3 const* p);

bool      physx_PxPlane_contains(physx_PxPlane const* self_, physx_PxVec3 const* p);

    /// projects p into the plane
physx_PxVec3      physx_PxPlane_project(physx_PxPlane const* self_, physx_PxVec3 const* p);

    /// find an arbitrary point in the plane
physx_PxVec3      physx_PxPlane_pointInPlane(physx_PxPlane const* self_);

    /// equivalent plane with unit normal
void      physx_PxPlane_normalize_mut(physx_PxPlane* self_);

    /// transform plane
physx_PxPlane      physx_PxPlane_transform(physx_PxPlane const* self_, physx_PxTransform const* pose);

    /// inverse-transform plane
physx_PxPlane      physx_PxPlane_inverseTransform(physx_PxPlane const* self_, physx_PxTransform const* pose);

    /// finds the shortest rotation between two vectors.
    ///
    /// a rotation about an axis normal to the two vectors which takes one to the other via the shortest path
physx_PxQuat      physx_phys_PxShortestRotation(physx_PxVec3 const* from, physx_PxVec3 const* target);

physx_PxVec3      physx_phys_PxDiagonalize(physx_PxMat33 const* m, physx_PxQuat* axes);

    /// creates a transform from the endpoints of a segment, suitable for an actor transform for a PxCapsuleGeometry
    ///
    /// A PxTransform which will transform the vector (1,0,0) to the capsule axis shrunk by the halfHeight
physx_PxTransform      physx_phys_PxTransformFromSegment(physx_PxVec3 const* p0, physx_PxVec3 const* p1, float* halfHeight);

    /// creates a transform from a plane equation, suitable for an actor transform for a PxPlaneGeometry
    ///
    /// a PxTransform which will transform the plane PxPlane(1,0,0,0) to the specified plane
physx_PxTransform      physx_phys_PxTransformFromPlaneEquation(physx_PxPlane const* plane);

    /// creates a plane equation from a transform, such as the actor transform for a PxPlaneGeometry
    ///
    /// the plane
physx_PxPlane      physx_phys_PxPlaneEquationFromTransform(physx_PxTransform const* pose);

    /// Spherical linear interpolation of two quaternions.
    ///
    /// Returns left when t=0, right when t=1 and a linear interpolation of left and right when 0
    /// <
    /// t
    /// <
    /// 1.
    /// Returns angle between -PI and PI in radians
physx_PxQuat      physx_phys_PxSlerp(float t, physx_PxQuat const* left, physx_PxQuat const* right);

    /// integrate transform.
void      physx_phys_PxIntegrateTransform(physx_PxTransform const* curTrans, physx_PxVec3 const* linvel, physx_PxVec3 const* angvel, float timeStep, physx_PxTransform* result);

    /// Compute the exponent of a PxVec3
physx_PxQuat      physx_phys_PxExp(physx_PxVec3 const* v);

    /// computes a oriented bounding box around the scaled basis.
    ///
    /// Bounding box extent.
physx_PxVec3      physx_phys_PxOptimizeBoundingBox(physx_PxMat33* basis);

    /// return Returns the log of a PxQuat
physx_PxVec3      physx_phys_PxLog(physx_PxQuat const* q);

    /// return Returns 0 if v.x is largest element of v, 1 if v.y is largest element, 2 if v.z is largest element.
uint32_t      physx_phys_PxLargestAxis(physx_PxVec3 const* v);

    /// Compute tan(theta/2) given sin(theta) and cos(theta) as inputs.
    ///
    /// Returns tan(theta/2)
float      physx_phys_PxTanHalf(float sin, float cos);

    /// Compute the closest point on an 2d ellipse to a given 2d point.
    ///
    /// Returns the 2d position on the surface of the ellipse that is closest to point.
physx_PxVec3      physx_phys_PxEllipseClamp(physx_PxVec3 const* point, physx_PxVec3 const* radii);

    /// Compute from an input quaternion q a pair of quaternions (swing, twist) such that
    /// q = swing * twist
    /// with the caveats that swing.x = twist.y = twist.z = 0.
void      physx_phys_PxSeparateSwingTwist(physx_PxQuat const* q, physx_PxQuat* swing, physx_PxQuat* twist);

    /// Compute the angle between two non-unit vectors
    ///
    /// Returns the angle (in radians) between the two vector v0 and v1.
float      physx_phys_PxComputeAngle(physx_PxVec3 const* v0, physx_PxVec3 const* v1);

    /// Compute two normalized vectors (right and up) that are perpendicular to an input normalized vector (dir).
void      physx_phys_PxComputeBasisVectors(physx_PxVec3 const* dir, physx_PxVec3* right, physx_PxVec3* up);

    /// Compute three normalized vectors (dir, right and up) that are parallel to (dir) and perpendicular to (right, up) the
    /// normalized direction vector (p1 - p0)/||p1 - p0||.
void      physx_phys_PxComputeBasisVectors_1(physx_PxVec3 const* p0, physx_PxVec3 const* p1, physx_PxVec3* dir, physx_PxVec3* right, physx_PxVec3* up);

    /// Compute (i+1)%3
uint32_t      physx_phys_PxGetNextIndex3(uint32_t i);

void      physx_phys_computeBarycentric(physx_PxVec3 const* a, physx_PxVec3 const* b, physx_PxVec3 const* c, physx_PxVec3 const* d, physx_PxVec3 const* p, physx_PxVec4* bary);

void      physx_phys_computeBarycentric_1(physx_PxVec3 const* a, physx_PxVec3 const* b, physx_PxVec3 const* c, physx_PxVec3 const* p, physx_PxVec4* bary);

float      physx_Interpolation_PxLerp(float a, float b, float t);

float      physx_Interpolation_PxBiLerp(float f00, float f10, float f01, float f11, float tx, float ty);

float      physx_Interpolation_PxTriLerp(float f000, float f100, float f010, float f110, float f001, float f101, float f011, float f111, float tx, float ty, float tz);

uint32_t      physx_Interpolation_PxSDFIdx(uint32_t i, uint32_t j, uint32_t k, uint32_t nbX, uint32_t nbY);

float      physx_Interpolation_PxSDFSampleImpl(float const* sdf, physx_PxVec3 const* localPos, physx_PxVec3 const* sdfBoxLower, physx_PxVec3 const* sdfBoxHigher, float sdfDx, float invSdfDx, uint32_t dimX, uint32_t dimY, uint32_t dimZ, float tolerance);

float      physx_phys_PxSdfSample(float const* sdf, physx_PxVec3 const* localPos, physx_PxVec3 const* sdfBoxLower, physx_PxVec3 const* sdfBoxHigher, float sdfDx, float invSdfDx, uint32_t dimX, uint32_t dimY, uint32_t dimZ, physx_PxVec3* gradient, float tolerance);

    /// The constructor for Mutex creates a mutex. It is initially unlocked.
physx_PxMutexImpl*      physx_PxMutexImpl_new_alloc();

    /// The destructor for Mutex deletes the mutex.
void      physx_PxMutexImpl_delete(physx_PxMutexImpl* self_);

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method blocks until the mutex is
    /// unlocked.
void      physx_PxMutexImpl_lock_mut(physx_PxMutexImpl* self_);

    /// Acquire (lock) the mutex. If the mutex is already locked
    /// by another thread, this method returns false without blocking.
bool      physx_PxMutexImpl_trylock_mut(physx_PxMutexImpl* self_);

    /// Release (unlock) the mutex.
void      physx_PxMutexImpl_unlock_mut(physx_PxMutexImpl* self_);

    /// Size of this class.
uint32_t      physx_PxMutexImpl_getSize();

physx_PxReadWriteLock*      physx_PxReadWriteLock_new_alloc();

void      physx_PxReadWriteLock_delete(physx_PxReadWriteLock* self_);

void      physx_PxReadWriteLock_lockReader_mut(physx_PxReadWriteLock* self_, bool takeLock);

void      physx_PxReadWriteLock_lockWriter_mut(physx_PxReadWriteLock* self_);

void      physx_PxReadWriteLock_unlockReader_mut(physx_PxReadWriteLock* self_);

void      physx_PxReadWriteLock_unlockWriter_mut(physx_PxReadWriteLock* self_);

    /// Mark the beginning of a nested profile block
    ///
    /// Returns implementation-specific profiler data for this event
void*      physx_PxProfilerCallback_zoneStart_mut(physx_PxProfilerCallback* self_, char const* eventName, bool detached, uint64_t contextId);

    /// Mark the end of a nested profile block
    ///
    /// eventName plus contextId can be used to uniquely match up start and end of a zone.
void      physx_PxProfilerCallback_zoneEnd_mut(physx_PxProfilerCallback* self_, void* profilerData, char const* eventName, bool detached, uint64_t contextId);

physx_PxProfileScoped*      physx_PxProfileScoped_new_alloc(physx_PxProfilerCallback* callback, char const* eventName, bool detached, uint64_t contextId);

void      physx_PxProfileScoped_delete(physx_PxProfileScoped* self_);

physx_PxSListEntry      physx_PxSListEntry_new();

physx_PxSListEntry*      physx_PxSListEntry_next_mut(physx_PxSListEntry* self_);

physx_PxSListImpl*      physx_PxSListImpl_new_alloc();

void      physx_PxSListImpl_delete(physx_PxSListImpl* self_);

void      physx_PxSListImpl_push_mut(physx_PxSListImpl* self_, physx_PxSListEntry* entry);

physx_PxSListEntry*      physx_PxSListImpl_pop_mut(physx_PxSListImpl* self_);

physx_PxSListEntry*      physx_PxSListImpl_flush_mut(physx_PxSListImpl* self_);

uint32_t      physx_PxSListImpl_getSize();

physx_PxSyncImpl*      physx_PxSyncImpl_new_alloc();

void      physx_PxSyncImpl_delete(physx_PxSyncImpl* self_);

    /// Wait on the object for at most the given number of ms. Returns
    /// true if the object is signaled. Sync::waitForever will block forever
    /// or until the object is signaled.
bool      physx_PxSyncImpl_wait_mut(physx_PxSyncImpl* self_, uint32_t milliseconds);

    /// Signal the synchronization object, waking all threads waiting on it
void      physx_PxSyncImpl_set_mut(physx_PxSyncImpl* self_);

    /// Reset the synchronization object
void      physx_PxSyncImpl_reset_mut(physx_PxSyncImpl* self_);

    /// Size of this class.
uint32_t      physx_PxSyncImpl_getSize();

physx_PxRunnable*      physx_PxRunnable_new_alloc();

void      physx_PxRunnable_delete(physx_PxRunnable* self_);

void      physx_PxRunnable_execute_mut(physx_PxRunnable* self_);

uint32_t      physx_phys_PxTlsAlloc();

void      physx_phys_PxTlsFree(uint32_t index);

void*      physx_phys_PxTlsGet(uint32_t index);

size_t      physx_phys_PxTlsGetValue(uint32_t index);

uint32_t      physx_phys_PxTlsSet(uint32_t index, void* value);

uint32_t      physx_phys_PxTlsSetValue(uint32_t index, size_t value);

physx_PxCounterFrequencyToTensOfNanos      physx_PxCounterFrequencyToTensOfNanos_new(uint64_t inNum, uint64_t inDenom);

uint64_t      physx_PxCounterFrequencyToTensOfNanos_toTensOfNanos(physx_PxCounterFrequencyToTensOfNanos const* self_, uint64_t inCounter);

physx_PxCounterFrequencyToTensOfNanos const*      physx_PxTime_getBootCounterFrequency();

physx_PxCounterFrequencyToTensOfNanos      physx_PxTime_getCounterFrequency();

uint64_t      physx_PxTime_getCurrentCounterValue();

uint64_t      physx_PxTime_getCurrentTimeInTensOfNanoSeconds();

physx_PxTime      physx_PxTime_new();

double      physx_PxTime_getElapsedSeconds_mut(physx_PxTime* self_);

double      physx_PxTime_peekElapsedSeconds_mut(physx_PxTime* self_);

double      physx_PxTime_getLastTime(physx_PxTime const* self_);

    /// default constructor leaves data uninitialized.
physx_PxVec2      physx_PxVec2_new();

    /// zero constructor.
physx_PxVec2      physx_PxVec2_new_1(int32_t anon_param0);

    /// Assigns scalar parameter to all elements.
    ///
    /// Useful to initialize to zero or one.
physx_PxVec2      physx_PxVec2_new_2(float a);

    /// Initializes from 2 scalar parameters.
physx_PxVec2      physx_PxVec2_new_3(float nx, float ny);

    /// tests for exact zero vector
bool      physx_PxVec2_isZero(physx_PxVec2 const* self_);

    /// returns true if all 2 elems of the vector are finite (not NAN or INF, etc.)
bool      physx_PxVec2_isFinite(physx_PxVec2 const* self_);

    /// is normalized - used by API parameter validation
bool      physx_PxVec2_isNormalized(physx_PxVec2 const* self_);

    /// returns the squared magnitude
    ///
    /// Avoids calling PxSqrt()!
float      physx_PxVec2_magnitudeSquared(physx_PxVec2 const* self_);

    /// returns the magnitude
float      physx_PxVec2_magnitude(physx_PxVec2 const* self_);

    /// returns the scalar product of this and other.
float      physx_PxVec2_dot(physx_PxVec2 const* self_, physx_PxVec2 const* v);

    /// returns a unit vector
physx_PxVec2      physx_PxVec2_getNormalized(physx_PxVec2 const* self_);

    /// normalizes the vector in place
float      physx_PxVec2_normalize_mut(physx_PxVec2* self_);

    /// a[i] * b[i], for all i.
physx_PxVec2      physx_PxVec2_multiply(physx_PxVec2 const* self_, physx_PxVec2 const* a);

    /// element-wise minimum
physx_PxVec2      physx_PxVec2_minimum(physx_PxVec2 const* self_, physx_PxVec2 const* v);

    /// returns MIN(x, y);
float      physx_PxVec2_minElement(physx_PxVec2 const* self_);

    /// element-wise maximum
physx_PxVec2      physx_PxVec2_maximum(physx_PxVec2 const* self_, physx_PxVec2 const* v);

    /// returns MAX(x, y);
float      physx_PxVec2_maxElement(physx_PxVec2 const* self_);

physx_PxStridedData      physx_PxStridedData_new();

physx_PxBoundedData      physx_PxBoundedData_new();

physx_PxDebugPoint      physx_PxDebugPoint_new(physx_PxVec3 const* p, uint32_t const* c);

physx_PxDebugLine      physx_PxDebugLine_new(physx_PxVec3 const* p0, physx_PxVec3 const* p1, uint32_t const* c);

physx_PxDebugTriangle      physx_PxDebugTriangle_new(physx_PxVec3 const* p0, physx_PxVec3 const* p1, physx_PxVec3 const* p2, uint32_t const* c);

physx_PxDebugText      physx_PxDebugText_new();

physx_PxDebugText      physx_PxDebugText_new_1(physx_PxVec3 const* pos, float const* sz, uint32_t const* clr, char const* str);

void      physx_PxRenderBuffer_delete(physx_PxRenderBuffer* self_);

uint32_t      physx_PxRenderBuffer_getNbPoints(physx_PxRenderBuffer const* self_);

physx_PxDebugPoint const*      physx_PxRenderBuffer_getPoints(physx_PxRenderBuffer const* self_);

void      physx_PxRenderBuffer_addPoint_mut(physx_PxRenderBuffer* self_, physx_PxDebugPoint const* point);

uint32_t      physx_PxRenderBuffer_getNbLines(physx_PxRenderBuffer const* self_);

physx_PxDebugLine const*      physx_PxRenderBuffer_getLines(physx_PxRenderBuffer const* self_);

void      physx_PxRenderBuffer_addLine_mut(physx_PxRenderBuffer* self_, physx_PxDebugLine const* line);

physx_PxDebugLine*      physx_PxRenderBuffer_reserveLines_mut(physx_PxRenderBuffer* self_, uint32_t nbLines);

physx_PxDebugPoint*      physx_PxRenderBuffer_reservePoints_mut(physx_PxRenderBuffer* self_, uint32_t nbLines);

uint32_t      physx_PxRenderBuffer_getNbTriangles(physx_PxRenderBuffer const* self_);

physx_PxDebugTriangle const*      physx_PxRenderBuffer_getTriangles(physx_PxRenderBuffer const* self_);

void      physx_PxRenderBuffer_addTriangle_mut(physx_PxRenderBuffer* self_, physx_PxDebugTriangle const* triangle);

void      physx_PxRenderBuffer_append_mut(physx_PxRenderBuffer* self_, physx_PxRenderBuffer const* other);

void      physx_PxRenderBuffer_clear_mut(physx_PxRenderBuffer* self_);

void      physx_PxRenderBuffer_shift_mut(physx_PxRenderBuffer* self_, physx_PxVec3 const* delta);

bool      physx_PxRenderBuffer_empty(physx_PxRenderBuffer const* self_);

void      physx_PxProcessPxBaseCallback_delete(physx_PxProcessPxBaseCallback* self_);

void      physx_PxProcessPxBaseCallback_process_mut(physx_PxProcessPxBaseCallback* self_, physx_PxBase* anon_param0);

    /// Registers a reference value corresponding to a PxBase object.
    ///
    /// This method is assumed to be called in the implementation of PxSerializer::registerReferences for serialized
    /// references that need to be resolved on deserialization.
    ///
    /// A reference needs to be associated with exactly one PxBase object in either the collection or the
    /// external references collection.
    ///
    /// Different kinds of references are supported and need to be specified. In the most common case
    /// (PX_SERIAL_REF_KIND_PXBASE) the PxBase object matches the reference value (which is the pointer
    /// to the PxBase object). Integer references maybe registered as well (used for internal material
    /// indices with PX_SERIAL_REF_KIND_MATERIAL_IDX). Other kinds could be added with the restriction that
    /// for pointer types the kind value needs to be marked with the PX_SERIAL_REF_KIND_PTR_TYPE_BIT.
void      physx_PxSerializationContext_registerReference_mut(physx_PxSerializationContext* self_, physx_PxBase* base, uint32_t kind, size_t reference);

    /// Returns the collection that is being serialized.
physx_PxCollection const*      physx_PxSerializationContext_getCollection(physx_PxSerializationContext const* self_);

    /// Serializes object data and object extra data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
void      physx_PxSerializationContext_writeData_mut(physx_PxSerializationContext* self_, void const* data, uint32_t size);

    /// Aligns the serialized data.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportData and PxSerializer::exportExtraData.
void      physx_PxSerializationContext_alignData_mut(physx_PxSerializationContext* self_, uint32_t alignment);

    /// Helper function to write a name to the extraData if serialization is configured to save names.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::exportExtraData.
void      physx_PxSerializationContext_writeName_mut(physx_PxSerializationContext* self_, char const* name);

    /// Retrieves a pointer to a deserialized PxBase object given a corresponding deserialized reference value
    ///
    /// This method is assumed to be called in the implementation of PxSerializer::createObject in order
    /// to update reference values on deserialization.
    ///
    /// To update a PxBase reference the corresponding deserialized pointer value needs to be provided in order to retrieve
    /// the location of the corresponding deserialized PxBase object. (PxDeserializationContext::translatePxBase simplifies
    /// this common case).
    ///
    /// For other kinds of references the reverence values need to be updated by deduction given the corresponding PxBase instance.
    ///
    /// PxBase object associated with the reference value
physx_PxBase*      physx_PxDeserializationContext_resolveReference(physx_PxDeserializationContext const* self_, uint32_t kind, size_t reference);

    /// Helper function to read a name from the extra data during deserialization.
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
void      physx_PxDeserializationContext_readName_mut(physx_PxDeserializationContext* self_, char const** name);

    /// Function to align the extra data stream to a power of 2 alignment
    ///
    /// This function is assumed to be called within the implementation of PxSerializer::createObject.
void      physx_PxDeserializationContext_alignExtraData_mut(physx_PxDeserializationContext* self_, uint32_t alignment);

    /// Register a serializer for a concrete type
void      physx_PxSerializationRegistry_registerSerializer_mut(physx_PxSerializationRegistry* self_, uint16_t type_, physx_PxSerializer* serializer);

    /// Unregister a serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered serializer corresponding to type, NULL for types for which no serializer has been registered.
physx_PxSerializer*      physx_PxSerializationRegistry_unregisterSerializer_mut(physx_PxSerializationRegistry* self_, uint16_t type_);

    /// Returns PxSerializer corresponding to type
    ///
    /// Registered PxSerializer object corresponding to type
physx_PxSerializer const*      physx_PxSerializationRegistry_getSerializer(physx_PxSerializationRegistry const* self_, uint16_t type_);

    /// Register a RepX serializer for a concrete type
void      physx_PxSerializationRegistry_registerRepXSerializer_mut(physx_PxSerializationRegistry* self_, uint16_t type_, physx_PxRepXSerializer* serializer);

    /// Unregister a RepX serializer for a concrete type, and retrieves the corresponding serializer object.
    ///
    /// Unregistered PxRepxSerializer corresponding to type, NULL for types for which no RepX serializer has been registered.
physx_PxRepXSerializer*      physx_PxSerializationRegistry_unregisterRepXSerializer_mut(physx_PxSerializationRegistry* self_, uint16_t type_);

    /// Returns RepX serializer given the corresponding type name
    ///
    /// Registered PxRepXSerializer object corresponding to type name
physx_PxRepXSerializer*      physx_PxSerializationRegistry_getRepXSerializer(physx_PxSerializationRegistry const* self_, char const* typeName);

    /// Releases PxSerializationRegistry instance.
    ///
    /// This unregisters all PhysX and PhysXExtension serializers. Make sure to unregister all custom type
    /// serializers before releasing the PxSerializationRegistry.
void      physx_PxSerializationRegistry_release_mut(physx_PxSerializationRegistry* self_);

    /// Adds a PxBase object to the collection.
    ///
    /// Adds a PxBase object to the collection. Optionally a PxSerialObjectId can be provided
    /// in order to resolve dependencies between collections. A PxSerialObjectId value of PX_SERIAL_OBJECT_ID_INVALID
    /// means the object remains without id. Objects can be added regardless of other objects they require. If the object
    /// is already in the collection, the ID will be set if it was PX_SERIAL_OBJECT_ID_INVALID previously, otherwise the
    /// operation fails.
void      physx_PxCollection_add_mut(physx_PxCollection* self_, physx_PxBase* object, uint64_t id);

    /// Removes a PxBase member object from the collection.
    ///
    /// Object needs to be contained by the collection.
void      physx_PxCollection_remove_mut(physx_PxCollection* self_, physx_PxBase* object);

    /// Returns whether the collection contains a certain PxBase object.
    ///
    /// Whether object is contained.
bool      physx_PxCollection_contains(physx_PxCollection const* self_, physx_PxBase* object);

    /// Adds an id to a member PxBase object.
    ///
    /// If the object is already associated with an id within the collection, the id is replaced.
    /// May only be called for objects that are members of the collection. The id needs to be unique
    /// within the collection.
void      physx_PxCollection_addId_mut(physx_PxCollection* self_, physx_PxBase* object, uint64_t id);

    /// Removes id from a contained PxBase object.
    ///
    /// May only be called for ids that are associated with an object in the collection.
void      physx_PxCollection_removeId_mut(physx_PxCollection* self_, uint64_t id);

    /// Adds all PxBase objects and their ids of collection to this collection.
    ///
    /// PxBase objects already in this collection are ignored. Object ids need to be conflict
    /// free, i.e. the same object may not have two different ids within the two collections.
void      physx_PxCollection_add_mut_1(physx_PxCollection* self_, physx_PxCollection* collection);

    /// Removes all PxBase objects of collection from this collection.
    ///
    /// PxBase objects not present in this collection are ignored. Ids of objects
    /// which are removed are also removed.
void      physx_PxCollection_remove_mut_1(physx_PxCollection* self_, physx_PxCollection* collection);

    /// Gets number of PxBase objects in this collection.
    ///
    /// Number of objects in this collection
uint32_t      physx_PxCollection_getNbObjects(physx_PxCollection const* self_);

    /// Gets the PxBase object of this collection given its index.
    ///
    /// PxBase object at index index
physx_PxBase*      physx_PxCollection_getObject(physx_PxCollection const* self_, uint32_t index);

    /// Copies member PxBase pointers to a user specified buffer.
    ///
    /// number of members PxBase objects that have been written to the userBuffer
uint32_t      physx_PxCollection_getObjects(physx_PxCollection const* self_, physx_PxBase** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Looks for a PxBase object given a PxSerialObjectId value.
    ///
    /// If there is no PxBase object in the collection with the given id, NULL is returned.
    ///
    /// PxBase object with the given id value or NULL
physx_PxBase*      physx_PxCollection_find(physx_PxCollection const* self_, uint64_t id);

    /// Gets number of PxSerialObjectId names in this collection.
    ///
    /// Number of PxSerialObjectId names in this collection
uint32_t      physx_PxCollection_getNbIds(physx_PxCollection const* self_);

    /// Copies member PxSerialObjectId values to a user specified buffer.
    ///
    /// number of members PxSerialObjectId values that have been written to the userBuffer
uint32_t      physx_PxCollection_getIds(physx_PxCollection const* self_, uint64_t* userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Gets the PxSerialObjectId name of a PxBase object within the collection.
    ///
    /// The PxBase object needs to be a member of the collection.
    ///
    /// PxSerialObjectId name of the object or PX_SERIAL_OBJECT_ID_INVALID if the object is unnamed
uint64_t      physx_PxCollection_getId(physx_PxCollection const* self_, physx_PxBase const* object);

    /// Deletes a collection object.
    ///
    /// This function only deletes the collection object, i.e. the container class. It doesn't delete objects
    /// that are part of the collection.
void      physx_PxCollection_release_mut(physx_PxCollection* self_);

    /// Creates a collection object.
    ///
    /// Objects can only be serialized or deserialized through a collection.
    /// For serialization, users must add objects to the collection and serialize the collection as a whole.
    /// For deserialization, the system gives back a collection of deserialized objects to users.
    ///
    /// The new collection object.
physx_PxCollection*      physx_phys_PxCreateCollection();

    /// Releases the PxBase instance, please check documentation of release in derived class.
void      physx_PxBase_release_mut(physx_PxBase* self_);

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
char const*      physx_PxBase_getConcreteTypeName(physx_PxBase const* self_);

    /// Returns concrete type of object.
    ///
    /// PxConcreteType::Enum of serialized object
uint16_t      physx_PxBase_getConcreteType(physx_PxBase const* self_);

    /// Set PxBaseFlag
void      physx_PxBase_setBaseFlag_mut(physx_PxBase* self_, int32_t flag, bool value);

    /// Set PxBaseFlags
void      physx_PxBase_setBaseFlags_mut(physx_PxBase* self_, uint16_t inFlags);

    /// Returns PxBaseFlags
    ///
    /// PxBaseFlags
uint16_t      physx_PxBase_getBaseFlags(physx_PxBase const* self_);

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
bool      physx_PxBase_isReleasable(physx_PxBase const* self_);

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
void      physx_PxRefCounted_release_mut(physx_PxRefCounted* self_);

    /// Returns the reference count of the object.
    ///
    /// At creation, the reference count of the object is 1. Every other object referencing this object increments the
    /// count by 1. When the reference count reaches 0, and only then, the object gets destroyed automatically.
    ///
    /// the current reference count.
uint32_t      physx_PxRefCounted_getReferenceCount(physx_PxRefCounted const* self_);

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
void      physx_PxRefCounted_acquireReference_mut(physx_PxRefCounted* self_);

    /// constructor sets to default
physx_PxTolerancesScale      physx_PxTolerancesScale_new(float defaultLength, float defaultSpeed);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid (returns always true).
bool      physx_PxTolerancesScale_isValid(physx_PxTolerancesScale const* self_);

    /// Allocate a new string.
    ///
    /// *Always* a valid null terminated string.  "" is returned if "" or null is passed in.
char const*      physx_PxStringTable_allocateStr_mut(physx_PxStringTable* self_, char const* inSrc);

    /// Release the string table and all the strings associated with it.
void      physx_PxStringTable_release_mut(physx_PxStringTable* self_);

    /// Returns string name of dynamic type.
    ///
    /// Class name of most derived type of this object.
char const*      physx_PxSerializer_getConcreteTypeName(physx_PxSerializer const* self_);

    /// Adds required objects to the collection.
    ///
    /// This method does not add the required objects recursively, e.g. objects required by required objects.
void      physx_PxSerializer_requiresObjects(physx_PxSerializer const* self_, physx_PxBase* anon_param0, physx_PxProcessPxBaseCallback* anon_param1);

    /// Whether the object is subordinate.
    ///
    /// A class is subordinate, if it can only be instantiated in the context of another class.
    ///
    /// Whether the class is subordinate
bool      physx_PxSerializer_isSubordinate(physx_PxSerializer const* self_);

    /// Exports object's extra data to stream.
void      physx_PxSerializer_exportExtraData(physx_PxSerializer const* self_, physx_PxBase* anon_param0, physx_PxSerializationContext* anon_param1);

    /// Exports object's data to stream.
void      physx_PxSerializer_exportData(physx_PxSerializer const* self_, physx_PxBase* anon_param0, physx_PxSerializationContext* anon_param1);

    /// Register references that the object maintains to other objects.
void      physx_PxSerializer_registerReferences(physx_PxSerializer const* self_, physx_PxBase* obj, physx_PxSerializationContext* s);

    /// Returns size needed to create the class instance.
    ///
    /// sizeof class instance.
size_t      physx_PxSerializer_getClassSize(physx_PxSerializer const* self_);

    /// Create object at a given address, resolve references and import extra data.
    ///
    /// Created PxBase pointer (needs to be identical to address before increment).
physx_PxBase*      physx_PxSerializer_createObject(physx_PxSerializer const* self_, uint8_t** address, physx_PxDeserializationContext* context);

    /// *******************************************************************************************************************
void      physx_PxSerializer_delete(physx_PxSerializer* self_);

    /// Builds object (TriangleMesh, Heightfield, ConvexMesh or BVH) from given data in PxPhysics.
    ///
    /// PxBase Created object in PxPhysics.
physx_PxBase*      physx_PxInsertionCallback_buildObjectFromData_mut(physx_PxInsertionCallback* self_, int32_t type_, void* data);

    /// Set the user-provided dispatcher object for CPU tasks
void      physx_PxTaskManager_setCpuDispatcher_mut(physx_PxTaskManager* self_, physx_PxCpuDispatcher* ref_);

    /// Get the user-provided dispatcher object for CPU tasks
    ///
    /// The CPU dispatcher object.
physx_PxCpuDispatcher*      physx_PxTaskManager_getCpuDispatcher(physx_PxTaskManager const* self_);

    /// Reset any dependencies between Tasks
    ///
    /// Will be called at the start of every frame before tasks are submitted.
void      physx_PxTaskManager_resetDependencies_mut(physx_PxTaskManager* self_);

    /// Called by the owning scene to start the task graph.
    ///
    /// All tasks with ref count of 1 will be dispatched.
void      physx_PxTaskManager_startSimulation_mut(physx_PxTaskManager* self_);

    /// Called by the owning scene at the end of a simulation step.
void      physx_PxTaskManager_stopSimulation_mut(physx_PxTaskManager* self_);

    /// Called by the worker threads to inform the PxTaskManager that a task has completed processing.
void      physx_PxTaskManager_taskCompleted_mut(physx_PxTaskManager* self_, physx_PxTask* task);

    /// Retrieve a task by name
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
uint32_t      physx_PxTaskManager_getNamedTask_mut(physx_PxTaskManager* self_, char const* name);

    /// Submit a task with a unique name.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
uint32_t      physx_PxTaskManager_submitNamedTask_mut(physx_PxTaskManager* self_, physx_PxTask* task, char const* name, int32_t type_);

    /// Submit an unnamed task.
    ///
    /// The ID of the task with that name, or eNOT_PRESENT if not found
uint32_t      physx_PxTaskManager_submitUnnamedTask_mut(physx_PxTaskManager* self_, physx_PxTask* task, int32_t type_);

    /// Retrieve a task given a task ID
    ///
    /// The task associated with the ID
physx_PxTask*      physx_PxTaskManager_getTaskFromID_mut(physx_PxTaskManager* self_, uint32_t id);

    /// Release the PxTaskManager object, referenced dispatchers will not be released
void      physx_PxTaskManager_release_mut(physx_PxTaskManager* self_);

    /// Construct a new PxTaskManager instance with the given [optional] dispatchers
physx_PxTaskManager*      physx_PxTaskManager_createTaskManager(physx_PxErrorCallback* errorCallback, physx_PxCpuDispatcher* anon_param1);

    /// Called by the TaskManager when a task is to be queued for execution.
    ///
    /// Upon receiving a task, the dispatcher should schedule the task to run.
    /// After the task has been run, it should call the release() method and
    /// discard its pointer.
void      physx_PxCpuDispatcher_submitTask_mut(physx_PxCpuDispatcher* self_, physx_PxBaseTask* task);

    /// Returns the number of available worker threads for this dispatcher.
    ///
    /// The SDK will use this count to control how many tasks are submitted. By
    /// matching the number of tasks with the number of execution units task
    /// overhead can be reduced.
uint32_t      physx_PxCpuDispatcher_getWorkerCount(physx_PxCpuDispatcher const* self_);

void      physx_PxCpuDispatcher_delete(physx_PxCpuDispatcher* self_);

    /// The user-implemented run method where the task's work should be performed
    ///
    /// run() methods must be thread safe, stack friendly (no alloca, etc), and
    /// must never block.
void      physx_PxBaseTask_run_mut(physx_PxBaseTask* self_);

    /// Return a user-provided task name for profiling purposes.
    ///
    /// It does not have to be unique, but unique names are helpful.
    ///
    /// The name of this task
char const*      physx_PxBaseTask_getName(physx_PxBaseTask const* self_);

    /// Implemented by derived implementation classes
void      physx_PxBaseTask_addReference_mut(physx_PxBaseTask* self_);

    /// Implemented by derived implementation classes
void      physx_PxBaseTask_removeReference_mut(physx_PxBaseTask* self_);

    /// Implemented by derived implementation classes
int32_t      physx_PxBaseTask_getReference(physx_PxBaseTask const* self_);

    /// Implemented by derived implementation classes
    ///
    /// A task may assume in its release() method that the task system no longer holds
    /// references to it - so it may safely run its destructor, recycle itself, etc.
    /// provided no additional user references to the task exist
void      physx_PxBaseTask_release_mut(physx_PxBaseTask* self_);

    /// Return PxTaskManager to which this task was submitted
    ///
    /// Note, can return NULL if task was not submitted, or has been
    /// completed.
physx_PxTaskManager*      physx_PxBaseTask_getTaskManager(physx_PxBaseTask const* self_);

void      physx_PxBaseTask_setContextId_mut(physx_PxBaseTask* self_, uint64_t id);

uint64_t      physx_PxBaseTask_getContextId(physx_PxBaseTask const* self_);

    /// Release method implementation
void      physx_PxTask_release_mut(physx_PxTask* self_);

    /// Inform the PxTaskManager this task must finish before the given
void      physx_PxTask_finishBefore_mut(physx_PxTask* self_, uint32_t taskID);

    /// Inform the PxTaskManager this task cannot start until the given
void      physx_PxTask_startAfter_mut(physx_PxTask* self_, uint32_t taskID);

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
void      physx_PxTask_addReference_mut(physx_PxTask* self_);

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
void      physx_PxTask_removeReference_mut(physx_PxTask* self_);

    /// Return the ref-count for this task
int32_t      physx_PxTask_getReference(physx_PxTask const* self_);

    /// Return the unique ID for this task
uint32_t      physx_PxTask_getTaskID(physx_PxTask const* self_);

    /// Called by PxTaskManager at submission time for initialization
    ///
    /// Perform simulation step initialization here.
void      physx_PxTask_submitted_mut(physx_PxTask* self_);

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// Submission is deferred until the task's mRefCount is decremented to zero.
    /// Note that we only use the PxTaskManager to query the appropriate dispatcher.
void      physx_PxLightCpuTask_setContinuation_mut(physx_PxLightCpuTask* self_, physx_PxTaskManager* tm, physx_PxBaseTask* c);

    /// Initialize this task and specify the task that will have its ref count decremented on completion.
    ///
    /// This overload of setContinuation() queries the PxTaskManager from the continuation
    /// task, which cannot be NULL.
void      physx_PxLightCpuTask_setContinuation_mut_1(physx_PxLightCpuTask* self_, physx_PxBaseTask* c);

    /// Retrieves continuation task
physx_PxBaseTask*      physx_PxLightCpuTask_getContinuation(physx_PxLightCpuTask const* self_);

    /// Manually decrement this task's reference count. If the reference
    /// count reaches zero, the task will be dispatched.
void      physx_PxLightCpuTask_removeReference_mut(physx_PxLightCpuTask* self_);

    /// Return the ref-count for this task
int32_t      physx_PxLightCpuTask_getReference(physx_PxLightCpuTask const* self_);

    /// Manually increment this task's reference count. The task will
    /// not be allowed to run until removeReference() is called.
void      physx_PxLightCpuTask_addReference_mut(physx_PxLightCpuTask* self_);

    /// called by CpuDispatcher after run method has completed
    ///
    /// Decrements the continuation task's reference count, if specified.
void      physx_PxLightCpuTask_release_mut(physx_PxLightCpuTask* self_);

    /// Returns the type of the geometry.
    ///
    /// The type of the object.
int32_t      physx_PxGeometry_getType(physx_PxGeometry const* self_);

    /// Constructor to initialize half extents from scalar parameters.
physx_PxBoxGeometry      physx_PxBoxGeometry_new(float hx, float hy, float hz);

    /// Constructor to initialize half extents from vector parameter.
physx_PxBoxGeometry      physx_PxBoxGeometry_new_1(physx_PxVec3 halfExtents_);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid box has a positive extent in each direction (halfExtents.x > 0, halfExtents.y > 0, halfExtents.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a box that has zero extent in any direction.
bool      physx_PxBoxGeometry_isValid(physx_PxBoxGeometry const* self_);

void      physx_PxBVHRaycastCallback_delete(physx_PxBVHRaycastCallback* self_);

bool      physx_PxBVHRaycastCallback_reportHit_mut(physx_PxBVHRaycastCallback* self_, uint32_t boundsIndex, float* distance);

void      physx_PxBVHOverlapCallback_delete(physx_PxBVHOverlapCallback* self_);

bool      physx_PxBVHOverlapCallback_reportHit_mut(physx_PxBVHOverlapCallback* self_, uint32_t boundsIndex);

void      physx_PxBVHTraversalCallback_delete(physx_PxBVHTraversalCallback* self_);

bool      physx_PxBVHTraversalCallback_visitNode_mut(physx_PxBVHTraversalCallback* self_, physx_PxBounds3 const* bounds);

bool      physx_PxBVHTraversalCallback_reportLeaf_mut(physx_PxBVHTraversalCallback* self_, uint32_t nbPrims, uint32_t const* prims);

    /// Raycast test against a BVH.
    ///
    /// false if query has been aborted
bool      physx_PxBVH_raycast(physx_PxBVH const* self_, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, float maxDist, physx_PxBVHRaycastCallback* cb, uint32_t queryFlags);

    /// Sweep test against a BVH.
    ///
    /// false if query has been aborted
bool      physx_PxBVH_sweep(physx_PxBVH const* self_, physx_PxGeometry const* geom, physx_PxTransform const* pose, physx_PxVec3 const* unitDir, float maxDist, physx_PxBVHRaycastCallback* cb, uint32_t queryFlags);

    /// Overlap test against a BVH.
    ///
    /// false if query has been aborted
bool      physx_PxBVH_overlap(physx_PxBVH const* self_, physx_PxGeometry const* geom, physx_PxTransform const* pose, physx_PxBVHOverlapCallback* cb, uint32_t queryFlags);

    /// Frustum culling test against a BVH.
    ///
    /// This is similar in spirit to an overlap query using a convex object around the frustum.
    /// However this specialized query has better performance, and can support more than the 6 planes
    /// of a frustum, which can be useful in portal-based engines.
    ///
    /// On the other hand this test only returns a conservative number of bounds, i.e. some of the returned
    /// bounds may actually be outside the frustum volume, close to it but not touching it. This is usually
    /// an ok performance trade-off when the function is used for view-frustum culling.
    ///
    /// false if query has been aborted
bool      physx_PxBVH_cull(physx_PxBVH const* self_, uint32_t nbPlanes, physx_PxPlane const* planes, physx_PxBVHOverlapCallback* cb, uint32_t queryFlags);

    /// Returns the number of bounds in the BVH.
    ///
    /// You can use [`getBounds`]() to retrieve the bounds.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
    ///
    /// Number of bounds in the BVH.
uint32_t      physx_PxBVH_getNbBounds(physx_PxBVH const* self_);

    /// Retrieve the read-only bounds in the BVH.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
physx_PxBounds3 const*      physx_PxBVH_getBounds(physx_PxBVH const* self_);

    /// Retrieve the bounds in the BVH.
    ///
    /// These bounds can be modified. Call refit() after modifications are done.
    ///
    /// These are the user-defined bounds passed to the BVH builder, not the internal bounds around each BVH node.
physx_PxBounds3*      physx_PxBVH_getBoundsForModification_mut(physx_PxBVH* self_);

    /// Refit the BVH.
    ///
    /// This function "refits" the tree, i.e. takes the new (leaf) bounding boxes into account and
    /// recomputes all the BVH bounds accordingly. This is an O(n) operation with n = number of bounds in the BVH.
    ///
    /// This works best with minor bounds modifications, i.e. when the bounds remain close to their initial values.
    /// With large modifications the tree quality degrades more and more, and subsequent query performance suffers.
    /// It might be a better strategy to create a brand new BVH if bounds change drastically.
    ///
    /// This function refits the whole tree after an arbitrary number of bounds have potentially been modified by
    /// users (via getBoundsForModification()). If you only have a small number of bounds to update, it might be
    /// more efficient to use setBounds() and partialRefit() instead.
void      physx_PxBVH_refit_mut(physx_PxBVH* self_);

    /// Update single bounds.
    ///
    /// This is an alternative to getBoundsForModification() / refit(). If you only have a small set of bounds to
    /// update, it can be inefficient to call the refit() function, because it refits the whole BVH.
    ///
    /// Instead, one can update individual bounds with this updateBounds() function. It sets the new bounds and
    /// marks the corresponding BVH nodes for partial refit. Once all the individual bounds have been updated,
    /// call partialRefit() to only refit the subset of marked nodes.
    ///
    /// true if success
bool      physx_PxBVH_updateBounds_mut(physx_PxBVH* self_, uint32_t boundsIndex, physx_PxBounds3 const* newBounds);

    /// Refits subset of marked nodes.
    ///
    /// This is an alternative to the refit() function, to be called after updateBounds() calls.
    /// See updateBounds() for details.
void      physx_PxBVH_partialRefit_mut(physx_PxBVH* self_);

    /// Generic BVH traversal function.
    ///
    /// This can be used to implement custom BVH traversal functions if provided ones are not enough.
    /// In particular this can be used to visualize the tree's bounds.
    ///
    /// false if query has been aborted
bool      physx_PxBVH_traverse(physx_PxBVH const* self_, physx_PxBVHTraversalCallback* cb);

char const*      physx_PxBVH_getConcreteTypeName(physx_PxBVH const* self_);

    /// Constructor, initializes to a capsule with passed radius and half height.
physx_PxCapsuleGeometry      physx_PxCapsuleGeometry_new(float radius_, float halfHeight_);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid.
    ///
    /// A valid capsule has radius > 0, halfHeight >= 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a capsule that has zero radius or height.
bool      physx_PxCapsuleGeometry_isValid(physx_PxCapsuleGeometry const* self_);

    /// Returns the number of vertices.
    ///
    /// Number of vertices.
uint32_t      physx_PxConvexMesh_getNbVertices(physx_PxConvexMesh const* self_);

    /// Returns the vertices.
    ///
    /// Array of vertices.
physx_PxVec3 const*      physx_PxConvexMesh_getVertices(physx_PxConvexMesh const* self_);

    /// Returns the index buffer.
    ///
    /// Index buffer.
uint8_t const*      physx_PxConvexMesh_getIndexBuffer(physx_PxConvexMesh const* self_);

    /// Returns the number of polygons.
    ///
    /// Number of polygons.
uint32_t      physx_PxConvexMesh_getNbPolygons(physx_PxConvexMesh const* self_);

    /// Returns the polygon data.
    ///
    /// True if success.
bool      physx_PxConvexMesh_getPolygonData(physx_PxConvexMesh const* self_, uint32_t index, physx_PxHullPolygon* data);

    /// Decrements the reference count of a convex mesh and releases it if the new reference count is zero.
void      physx_PxConvexMesh_release_mut(physx_PxConvexMesh* self_);

    /// Returns the mass properties of the mesh assuming unit density.
    ///
    /// The following relationship holds between mass and volume:
    ///
    /// mass = volume * density
    ///
    /// The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.
    ///
    /// Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
    /// localInertia of the unit density mesh by d.
void      physx_PxConvexMesh_getMassInformation(physx_PxConvexMesh const* self_, float* mass, physx_PxMat33* localInertia, physx_PxVec3* localCenterOfMass);

    /// Returns the local-space (vertex space) AABB from the convex mesh.
    ///
    /// local-space bounds
physx_PxBounds3      physx_PxConvexMesh_getLocalBounds(physx_PxConvexMesh const* self_);

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
float const*      physx_PxConvexMesh_getSDF(physx_PxConvexMesh const* self_);

char const*      physx_PxConvexMesh_getConcreteTypeName(physx_PxConvexMesh const* self_);

    /// This method decides whether a convex mesh is gpu compatible. If the total number of vertices are more than 64 or any number of vertices in a polygon is more than 32, or
    /// convex hull data was not cooked with GPU data enabled during cooking or was loaded from a serialized collection, the convex hull is incompatible with GPU collision detection. Otherwise
    /// it is compatible.
    ///
    /// True if the convex hull is gpu compatible
bool      physx_PxConvexMesh_isGpuCompatible(physx_PxConvexMesh const* self_);

    /// Constructor initializes to identity scale.
physx_PxMeshScale      physx_PxMeshScale_new();

    /// Constructor from scalar.
physx_PxMeshScale      physx_PxMeshScale_new_1(float r);

    /// Constructor to initialize to arbitrary scale and identity scale rotation.
physx_PxMeshScale      physx_PxMeshScale_new_2(physx_PxVec3 const* s);

    /// Constructor to initialize to arbitrary scaling.
physx_PxMeshScale      physx_PxMeshScale_new_3(physx_PxVec3 const* s, physx_PxQuat const* r);

    /// Returns true if the scaling is an identity transformation.
bool      physx_PxMeshScale_isIdentity(physx_PxMeshScale const* self_);

    /// Returns the inverse of this scaling transformation.
physx_PxMeshScale      physx_PxMeshScale_getInverse(physx_PxMeshScale const* self_);

    /// Converts this transformation to a 3x3 matrix representation.
physx_PxMat33      physx_PxMeshScale_toMat33(physx_PxMeshScale const* self_);

    /// Returns true if combination of negative scale components will cause the triangle normal to flip. The SDK will flip the normals internally.
bool      physx_PxMeshScale_hasNegativeDeterminant(physx_PxMeshScale const* self_);

physx_PxVec3      physx_PxMeshScale_transform(physx_PxMeshScale const* self_, physx_PxVec3 const* v);

bool      physx_PxMeshScale_isValidForTriangleMesh(physx_PxMeshScale const* self_);

bool      physx_PxMeshScale_isValidForConvexMesh(physx_PxMeshScale const* self_);

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
physx_PxConvexMeshGeometry      physx_PxConvexMeshGeometry_new(physx_PxConvexMesh* mesh, physx_PxMeshScale const* scaling, uint8_t flags);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid convex mesh has a positive scale value in each direction (scale.x > 0, scale.y > 0, scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a convex that has zero extent in any direction.
bool      physx_PxConvexMeshGeometry_isValid(physx_PxConvexMeshGeometry const* self_);

    /// Constructor.
physx_PxSphereGeometry      physx_PxSphereGeometry_new(float ir);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid sphere has radius > 0.
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a sphere that has zero radius.
bool      physx_PxSphereGeometry_isValid(physx_PxSphereGeometry const* self_);

    /// Constructor.
physx_PxPlaneGeometry      physx_PxPlaneGeometry_new();

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
bool      physx_PxPlaneGeometry_isValid(physx_PxPlaneGeometry const* self_);

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
physx_PxTriangleMeshGeometry      physx_PxTriangleMeshGeometry_new(physx_PxTriangleMesh* mesh, physx_PxMeshScale const* scaling, uint8_t flags);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid triangle mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a triangle mesh that has zero extents in any direction.
bool      physx_PxTriangleMeshGeometry_isValid(physx_PxTriangleMeshGeometry const* self_);

    /// Constructor.
physx_PxHeightFieldGeometry      physx_PxHeightFieldGeometry_new(physx_PxHeightField* hf, uint8_t flags, float heightScale_, float rowScale_, float columnScale_);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid
    ///
    /// A valid height field has a positive scale value in each direction (heightScale > 0, rowScale > 0, columnScale > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a height field that has zero extents in any direction.
bool      physx_PxHeightFieldGeometry_isValid(physx_PxHeightFieldGeometry const* self_);

    /// Default constructor.
    ///
    /// Creates an empty object with no particles.
physx_PxParticleSystemGeometry      physx_PxParticleSystemGeometry_new();

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
bool      physx_PxParticleSystemGeometry_isValid(physx_PxParticleSystemGeometry const* self_);

    /// Default constructor.
physx_PxHairSystemGeometry      physx_PxHairSystemGeometry_new();

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
bool      physx_PxHairSystemGeometry_isValid(physx_PxHairSystemGeometry const* self_);

    /// Constructor. By default creates an empty object with a NULL mesh and identity scale.
physx_PxTetrahedronMeshGeometry      physx_PxTetrahedronMeshGeometry_new(physx_PxTetrahedronMesh* mesh);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
    ///
    /// A valid tetrahedron mesh has a positive scale value in each direction (scale.scale.x > 0, scale.scale.y > 0, scale.scale.z > 0).
    /// It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a tetrahedron mesh that has zero extents in any direction.
bool      physx_PxTetrahedronMeshGeometry_isValid(physx_PxTetrahedronMeshGeometry const* self_);

physx_PxQueryHit      physx_PxQueryHit_new();

physx_PxLocationHit      physx_PxLocationHit_new();

    /// For raycast hits: true for shapes overlapping with raycast origin.
    ///
    /// For sweep hits: true for shapes overlapping at zero sweep distance.
bool      physx_PxLocationHit_hadInitialOverlap(physx_PxLocationHit const* self_);

physx_PxGeomRaycastHit      physx_PxGeomRaycastHit_new();

physx_PxGeomOverlapHit      physx_PxGeomOverlapHit_new();

physx_PxGeomSweepHit      physx_PxGeomSweepHit_new();

physx_PxGeomIndexPair      physx_PxGeomIndexPair_new();

physx_PxGeomIndexPair      physx_PxGeomIndexPair_new_1(uint32_t _id0, uint32_t _id1);

    /// For internal use
uint32_t      physx_phys_PxCustomGeometry_getUniqueID();

    /// Default constructor
physx_PxCustomGeometryType      physx_PxCustomGeometryType_new();

    /// Invalid type
physx_PxCustomGeometryType      physx_PxCustomGeometryType_INVALID();

    /// Return custom type. The type purpose is for user to differentiate custom geometries. Not used by PhysX.
    ///
    /// Unique ID of a custom geometry type.
    ///
    /// User should use DECLARE_CUSTOM_GEOMETRY_TYPE and IMPLEMENT_CUSTOM_GEOMETRY_TYPE intead of overwriting this function.
physx_PxCustomGeometryType      physx_PxCustomGeometryCallbacks_getCustomType(physx_PxCustomGeometryCallbacks const* self_);

    /// Return local bounds.
    ///
    /// Bounding box in the geometry local space.
physx_PxBounds3      physx_PxCustomGeometryCallbacks_getLocalBounds(physx_PxCustomGeometryCallbacks const* self_, physx_PxGeometry const* geometry);

    /// Raycast. Cast a ray against the geometry in given pose.
    ///
    /// Number of hits.
uint32_t      physx_PxCustomGeometryCallbacks_raycast(physx_PxCustomGeometryCallbacks const* self_, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, physx_PxGeometry const* geom, physx_PxTransform const* pose, float maxDist, uint16_t hitFlags, uint32_t maxHits, physx_PxGeomRaycastHit* rayHits, uint32_t stride, physx_PxQueryThreadContext* threadContext);

    /// Overlap. Test if geometries overlap.
    ///
    /// True if there is overlap. False otherwise.
bool      physx_PxCustomGeometryCallbacks_overlap(physx_PxCustomGeometryCallbacks const* self_, physx_PxGeometry const* geom0, physx_PxTransform const* pose0, physx_PxGeometry const* geom1, physx_PxTransform const* pose1, physx_PxQueryThreadContext* threadContext);

    /// Sweep. Sweep one geometry against the other.
    ///
    /// True if there is hit. False otherwise.
bool      physx_PxCustomGeometryCallbacks_sweep(physx_PxCustomGeometryCallbacks const* self_, physx_PxVec3 const* unitDir, float maxDist, physx_PxGeometry const* geom0, physx_PxTransform const* pose0, physx_PxGeometry const* geom1, physx_PxTransform const* pose1, physx_PxGeomSweepHit* sweepHit, uint16_t hitFlags, float inflation, physx_PxQueryThreadContext* threadContext);

    /// Compute custom geometry mass properties. For geometries usable with dynamic rigidbodies.
void      physx_PxCustomGeometryCallbacks_computeMassProperties(physx_PxCustomGeometryCallbacks const* self_, physx_PxGeometry const* geometry, physx_PxMassProperties* massProperties);

    /// Compatible with PhysX's PCM feature. Allows to optimize contact generation.
bool      physx_PxCustomGeometryCallbacks_usePersistentContactManifold(physx_PxCustomGeometryCallbacks const* self_, physx_PxGeometry const* geometry, float* breakingThreshold);

void      physx_PxCustomGeometryCallbacks_delete(physx_PxCustomGeometryCallbacks* self_);

    /// Default constructor.
    ///
    /// Creates an empty object with a NULL callbacks pointer.
physx_PxCustomGeometry      physx_PxCustomGeometry_new();

    /// Constructor.
physx_PxCustomGeometry      physx_PxCustomGeometry_new_1(physx_PxCustomGeometryCallbacks* _callbacks);

    /// Returns true if the geometry is valid.
    ///
    /// True if the current settings are valid for shape creation.
bool      physx_PxCustomGeometry_isValid(physx_PxCustomGeometry const* self_);

    /// Returns the custom type of the custom geometry.
physx_PxCustomGeometryType      physx_PxCustomGeometry_getCustomType(physx_PxCustomGeometry const* self_);

int32_t      physx_PxGeometryHolder_getType(physx_PxGeometryHolder const* self_);

physx_PxGeometry*      physx_PxGeometryHolder_any_mut(physx_PxGeometryHolder* self_);

physx_PxGeometry const*      physx_PxGeometryHolder_any(physx_PxGeometryHolder const* self_);

physx_PxSphereGeometry*      physx_PxGeometryHolder_sphere_mut(physx_PxGeometryHolder* self_);

physx_PxSphereGeometry const*      physx_PxGeometryHolder_sphere(physx_PxGeometryHolder const* self_);

physx_PxPlaneGeometry*      physx_PxGeometryHolder_plane_mut(physx_PxGeometryHolder* self_);

physx_PxPlaneGeometry const*      physx_PxGeometryHolder_plane(physx_PxGeometryHolder const* self_);

physx_PxCapsuleGeometry*      physx_PxGeometryHolder_capsule_mut(physx_PxGeometryHolder* self_);

physx_PxCapsuleGeometry const*      physx_PxGeometryHolder_capsule(physx_PxGeometryHolder const* self_);

physx_PxBoxGeometry*      physx_PxGeometryHolder_box_mut(physx_PxGeometryHolder* self_);

physx_PxBoxGeometry const*      physx_PxGeometryHolder_box(physx_PxGeometryHolder const* self_);

physx_PxConvexMeshGeometry*      physx_PxGeometryHolder_convexMesh_mut(physx_PxGeometryHolder* self_);

physx_PxConvexMeshGeometry const*      physx_PxGeometryHolder_convexMesh(physx_PxGeometryHolder const* self_);

physx_PxTetrahedronMeshGeometry*      physx_PxGeometryHolder_tetMesh_mut(physx_PxGeometryHolder* self_);

physx_PxTetrahedronMeshGeometry const*      physx_PxGeometryHolder_tetMesh(physx_PxGeometryHolder const* self_);

physx_PxTriangleMeshGeometry*      physx_PxGeometryHolder_triangleMesh_mut(physx_PxGeometryHolder* self_);

physx_PxTriangleMeshGeometry const*      physx_PxGeometryHolder_triangleMesh(physx_PxGeometryHolder const* self_);

physx_PxHeightFieldGeometry*      physx_PxGeometryHolder_heightField_mut(physx_PxGeometryHolder* self_);

physx_PxHeightFieldGeometry const*      physx_PxGeometryHolder_heightField(physx_PxGeometryHolder const* self_);

physx_PxParticleSystemGeometry*      physx_PxGeometryHolder_particleSystem_mut(physx_PxGeometryHolder* self_);

physx_PxParticleSystemGeometry const*      physx_PxGeometryHolder_particleSystem(physx_PxGeometryHolder const* self_);

physx_PxHairSystemGeometry*      physx_PxGeometryHolder_hairSystem_mut(physx_PxGeometryHolder* self_);

physx_PxHairSystemGeometry const*      physx_PxGeometryHolder_hairSystem(physx_PxGeometryHolder const* self_);

physx_PxCustomGeometry*      physx_PxGeometryHolder_custom_mut(physx_PxGeometryHolder* self_);

physx_PxCustomGeometry const*      physx_PxGeometryHolder_custom(physx_PxGeometryHolder const* self_);

void      physx_PxGeometryHolder_storeAny_mut(physx_PxGeometryHolder* self_, physx_PxGeometry const* geometry);

physx_PxGeometryHolder      physx_PxGeometryHolder_new();

physx_PxGeometryHolder      physx_PxGeometryHolder_new_1(physx_PxGeometry const* geometry);

    /// Raycast test against a geometry object.
    ///
    /// All geometry types are supported except PxParticleSystemGeometry, PxTetrahedronMeshGeometry and PxHairSystemGeometry.
    ///
    /// Number of hits between the ray and the geometry object
uint32_t      physx_PxGeometryQuery_raycast(physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, physx_PxGeometry const* geom, physx_PxTransform const* pose, float maxDist, uint16_t hitFlags, uint32_t maxHits, physx_PxGeomRaycastHit* rayHits, uint32_t stride, uint32_t queryFlags, physx_PxQueryThreadContext* threadContext);

    /// Overlap test for two geometry objects.
    ///
    /// All combinations are supported except:
    ///
    /// PxPlaneGeometry vs. {PxPlaneGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxTriangleMeshGeometry vs. PxHeightFieldGeometry
    ///
    /// PxHeightFieldGeometry vs. PxHeightFieldGeometry
    ///
    /// Anything involving PxParticleSystemGeometry, PxTetrahedronMeshGeometry or PxHairSystemGeometry.
    ///
    /// True if the two geometry objects overlap
bool      physx_PxGeometryQuery_overlap(physx_PxGeometry const* geom0, physx_PxTransform const* pose0, physx_PxGeometry const* geom1, physx_PxTransform const* pose1, uint32_t queryFlags, physx_PxQueryThreadContext* threadContext);

    /// Sweep a specified geometry object in space and test for collision with a given object.
    ///
    /// The following combinations are supported.
    ///
    /// PxSphereGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxCapsuleGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxBoxGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// PxConvexMeshGeometry vs. {PxSphereGeometry, PxPlaneGeometry, PxCapsuleGeometry, PxBoxGeometry, PxConvexMeshGeometry, PxTriangleMeshGeometry, PxHeightFieldGeometry}
    ///
    /// True if the swept geometry object geom0 hits the object geom1
bool      physx_PxGeometryQuery_sweep(physx_PxVec3 const* unitDir, float maxDist, physx_PxGeometry const* geom0, physx_PxTransform const* pose0, physx_PxGeometry const* geom1, physx_PxTransform const* pose1, physx_PxGeomSweepHit* sweepHit, uint16_t hitFlags, float inflation, uint32_t queryFlags, physx_PxQueryThreadContext* threadContext);

    /// Compute minimum translational distance (MTD) between two geometry objects.
    ///
    /// All combinations of geom objects are supported except:
    /// - plane/plane
    /// - plane/mesh
    /// - plane/heightfield
    /// - mesh/mesh
    /// - mesh/heightfield
    /// - heightfield/heightfield
    /// - anything involving PxParticleSystemGeometry, PxTetrahedronMeshGeometry or PxHairSystemGeometry
    ///
    /// The function returns a unit vector ('direction') and a penetration depth ('depth').
    ///
    /// The depenetration vector D = direction * depth should be applied to the first object, to
    /// get out of the second object.
    ///
    /// Returned depth should always be positive or null.
    ///
    /// If objects do not overlap, the function can not compute the MTD and returns false.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
bool      physx_PxGeometryQuery_computePenetration(physx_PxVec3* direction, float* depth, physx_PxGeometry const* geom0, physx_PxTransform const* pose0, physx_PxGeometry const* geom1, physx_PxTransform const* pose1, uint32_t queryFlags);

    /// Computes distance between a point and a geometry object.
    ///
    /// Currently supported geometry objects: box, sphere, capsule, convex, mesh.
    ///
    /// For meshes, only the BVH34 midphase data-structure is supported.
    ///
    /// Square distance between the point and the geom object, or 0.0 if the point is inside the object, or -1.0 if an error occured (geometry type is not supported, or invalid pose)
float      physx_PxGeometryQuery_pointDistance(physx_PxVec3 const* point, physx_PxGeometry const* geom, physx_PxTransform const* pose, physx_PxVec3* closestPoint, uint32_t* closestIndex, uint32_t queryFlags);

    /// computes the bounds for a geometry object
void      physx_PxGeometryQuery_computeGeomBounds(physx_PxBounds3* bounds, physx_PxGeometry const* geom, physx_PxTransform const* pose, float offset, float inflation, uint32_t queryFlags);

    /// Checks if provided geometry is valid.
    ///
    /// True if geometry is valid.
bool      physx_PxGeometryQuery_isValid(physx_PxGeometry const* geom);

uint8_t      physx_PxHeightFieldSample_tessFlag(physx_PxHeightFieldSample const* self_);

void      physx_PxHeightFieldSample_setTessFlag_mut(physx_PxHeightFieldSample* self_);

void      physx_PxHeightFieldSample_clearTessFlag_mut(physx_PxHeightFieldSample* self_);

    /// Decrements the reference count of a height field and releases it if the new reference count is zero.
void      physx_PxHeightField_release_mut(physx_PxHeightField* self_);

    /// Writes out the sample data array.
    ///
    /// The user provides destBufferSize bytes storage at destBuffer.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// The number of bytes written.
uint32_t      physx_PxHeightField_saveCells(physx_PxHeightField const* self_, void* destBuffer, uint32_t destBufferSize);

    /// Replaces a rectangular subfield in the sample data array.
    ///
    /// The user provides the description of a rectangular subfield in subfieldDesc.
    /// The data is formatted and arranged as PxHeightFieldDesc.samples.
    ///
    /// True on success, false on failure. Failure can occur due to format mismatch.
    ///
    /// Modified samples are constrained to the same height quantization range as the original heightfield.
    /// Source samples that are out of range of target heightfield will be clipped with no error.
    /// PhysX does not keep a mapping from the heightfield to heightfield shapes that reference it.
    /// Call PxShape::setGeometry on each shape which references the height field, to ensure that internal data structures are updated to reflect the new geometry.
    /// Please note that PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
bool      physx_PxHeightField_modifySamples_mut(physx_PxHeightField* self_, int32_t startCol, int32_t startRow, physx_PxHeightFieldDesc const* subfieldDesc, bool shrinkBounds);

    /// Retrieves the number of sample rows in the samples array.
    ///
    /// The number of sample rows in the samples array.
uint32_t      physx_PxHeightField_getNbRows(physx_PxHeightField const* self_);

    /// Retrieves the number of sample columns in the samples array.
    ///
    /// The number of sample columns in the samples array.
uint32_t      physx_PxHeightField_getNbColumns(physx_PxHeightField const* self_);

    /// Retrieves the format of the sample data.
    ///
    /// The format of the sample data.
int32_t      physx_PxHeightField_getFormat(physx_PxHeightField const* self_);

    /// Retrieves the offset in bytes between consecutive samples in the array.
    ///
    /// The offset in bytes between consecutive samples in the array.
uint32_t      physx_PxHeightField_getSampleStride(physx_PxHeightField const* self_);

    /// Retrieves the convex edge threshold.
    ///
    /// The convex edge threshold.
float      physx_PxHeightField_getConvexEdgeThreshold(physx_PxHeightField const* self_);

    /// Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.
    ///
    /// The flags bits, combined from values of the enum ::PxHeightFieldFlag.
uint16_t      physx_PxHeightField_getFlags(physx_PxHeightField const* self_);

    /// Retrieves the height at the given coordinates in grid space.
    ///
    /// The height at the given coordinates or 0 if the coordinates are out of range.
float      physx_PxHeightField_getHeight(physx_PxHeightField const* self_, float x, float z);

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
uint16_t      physx_PxHeightField_getTriangleMaterialIndex(physx_PxHeightField const* self_, uint32_t triangleIndex);

    /// Returns a triangle face normal for a given triangle index
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Triangle normal for a given triangle index
physx_PxVec3      physx_PxHeightField_getTriangleNormal(physx_PxHeightField const* self_, uint32_t triangleIndex);

    /// Returns heightfield sample of given row and column
    ///
    /// Heightfield sample
physx_PxHeightFieldSample const*      physx_PxHeightField_getSample(physx_PxHeightField const* self_, uint32_t row, uint32_t column);

    /// Returns the number of times the heightfield data has been modified
    ///
    /// This method returns the number of times modifySamples has been called on this heightfield, so that code that has
    /// retained state that depends on the heightfield can efficiently determine whether it has been modified.
    ///
    /// the number of times the heightfield sample data has been modified.
uint32_t      physx_PxHeightField_getTimestamp(physx_PxHeightField const* self_);

char const*      physx_PxHeightField_getConcreteTypeName(physx_PxHeightField const* self_);

    /// Constructor sets to default.
physx_PxHeightFieldDesc      physx_PxHeightFieldDesc_new();

    /// (re)sets the structure to the default.
void      physx_PxHeightFieldDesc_setToDefault_mut(physx_PxHeightFieldDesc* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid.
bool      physx_PxHeightFieldDesc_isValid(physx_PxHeightFieldDesc const* self_);

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapTriangleMesh`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
void      physx_PxMeshQuery_getTriangle(physx_PxTriangleMeshGeometry const* triGeom, physx_PxTransform const* transform, uint32_t triangleIndex, physx_PxTriangle* triangle, uint32_t* vertexIndices, uint32_t* adjacencyIndices);

    /// Retrieves triangle data from a triangle ID.
    ///
    /// This function can be used together with [`findOverlapHeightField`]() to retrieve triangle properties.
    ///
    /// This function will flip the triangle normal whenever triGeom.scale.hasNegativeDeterminant() is true.
    ///
    /// TriangleIndex is an index used in internal format, which does have an index out of the bounds in last row.
    /// To traverse all tri indices in the HF, the following code can be applied:
    /// for (PxU32 row = 0; row
    /// <
    /// (nbRows - 1); row++)
    /// {
    /// for (PxU32 col = 0; col
    /// <
    /// (nbCols - 1); col++)
    /// {
    /// for (PxU32 k = 0; k
    /// <
    /// 2; k++)
    /// {
    /// const PxU32 triIndex = 2 * (row*nbCols + col) + k;
    /// ....
    /// }
    /// }
    /// }
void      physx_PxMeshQuery_getTriangle_1(physx_PxHeightFieldGeometry const* hfGeom, physx_PxTransform const* transform, uint32_t triangleIndex, physx_PxTriangle* triangle, uint32_t* vertexIndices, uint32_t* adjacencyIndices);

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// For mesh-vs-mesh overlap tests, please use the specialized function below.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
uint32_t      physx_PxMeshQuery_findOverlapTriangleMesh(physx_PxGeometry const* geom, physx_PxTransform const* geomPose, physx_PxTriangleMeshGeometry const* meshGeom, physx_PxTransform const* meshPose, uint32_t* results, uint32_t maxResults, uint32_t startIndex, bool* overflow, uint32_t queryFlags);

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Returned triangle indices can be used with [`getTriangle`]() to retrieve the triangle properties.
    ///
    /// Number of overlaps found, i.e. number of elements written to the results buffer
uint32_t      physx_PxMeshQuery_findOverlapHeightField(physx_PxGeometry const* geom, physx_PxTransform const* geomPose, physx_PxHeightFieldGeometry const* hfGeom, physx_PxTransform const* hfPose, uint32_t* results, uint32_t maxResults, uint32_t startIndex, bool* overflow, uint32_t queryFlags);

    /// Sweep a specified geometry object in space and test for collision with a set of given triangles.
    ///
    /// This function simply sweeps input geometry against each input triangle, in the order they are given.
    /// This is an O(N) operation with N = number of input triangles. It does not use any particular acceleration structure.
    ///
    /// True if the swept geometry object hits the specified triangles
    ///
    /// Only the following geometry types are currently supported: PxSphereGeometry, PxCapsuleGeometry, PxBoxGeometry
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// This function returns a single closest hit across all the input triangles. Multiple hits are not supported.
    ///
    /// Supported hitFlags are PxHitFlag::eDEFAULT, PxHitFlag::eASSUME_NO_INITIAL_OVERLAP, PxHitFlag::ePRECISE_SWEEP, PxHitFlag::eMESH_BOTH_SIDES, PxHitFlag::eMESH_ANY.
    ///
    /// ePOSITION is only defined when there is no initial overlap (sweepHit.hadInitialOverlap() == false)
    ///
    /// The returned normal for initially overlapping sweeps is set to -unitDir.
    ///
    /// Otherwise the returned normal is the front normal of the triangle even if PxHitFlag::eMESH_BOTH_SIDES is set.
    ///
    /// The returned PxGeomSweepHit::faceIndex parameter will hold the index of the hit triangle in input array, i.e. the range is [0; triangleCount). For initially overlapping sweeps, this is the index of overlapping triangle.
    ///
    /// The inflation parameter is not compatible with PxHitFlag::ePRECISE_SWEEP.
bool      physx_PxMeshQuery_sweep(physx_PxVec3 const* unitDir, float distance, physx_PxGeometry const* geom, physx_PxTransform const* pose, uint32_t triangleCount, physx_PxTriangle const* triangles, physx_PxGeomSweepHit* sweepHit, uint16_t hitFlags, uint32_t const* cachedIndex, float inflation, bool doubleSided, uint32_t queryFlags);

    /// constructor sets to default.
physx_PxSimpleTriangleMesh      physx_PxSimpleTriangleMesh_new();

    /// (re)sets the structure to the default.
void      physx_PxSimpleTriangleMesh_setToDefault_mut(physx_PxSimpleTriangleMesh* self_);

    /// returns true if the current settings are valid
bool      physx_PxSimpleTriangleMesh_isValid(physx_PxSimpleTriangleMesh const* self_);

    /// Constructor
physx_PxTriangle*      physx_PxTriangle_new_alloc();

    /// Constructor
physx_PxTriangle*      physx_PxTriangle_new_alloc_1(physx_PxVec3 const* p0, physx_PxVec3 const* p1, physx_PxVec3 const* p2);

    /// Destructor
void      physx_PxTriangle_delete(physx_PxTriangle* self_);

    /// Compute the normal of the Triangle.
void      physx_PxTriangle_normal(physx_PxTriangle const* self_, physx_PxVec3* _normal);

    /// Compute the unnormalized normal of the triangle.
void      physx_PxTriangle_denormalizedNormal(physx_PxTriangle const* self_, physx_PxVec3* _normal);

    /// Compute the area of the triangle.
    ///
    /// Area of the triangle.
float      physx_PxTriangle_area(physx_PxTriangle const* self_);

    /// Computes a point on the triangle from u and v barycentric coordinates.
physx_PxVec3      physx_PxTriangle_pointFromUV(physx_PxTriangle const* self_, float u, float v);

physx_PxTrianglePadded*      physx_PxTrianglePadded_new_alloc();

void      physx_PxTrianglePadded_delete(physx_PxTrianglePadded* self_);

    /// Returns the number of vertices.
    ///
    /// number of vertices
uint32_t      physx_PxTriangleMesh_getNbVertices(physx_PxTriangleMesh const* self_);

    /// Returns the vertices.
    ///
    /// array of vertices
physx_PxVec3 const*      physx_PxTriangleMesh_getVertices(physx_PxTriangleMesh const* self_);

    /// Returns all mesh vertices for modification.
    ///
    /// This function will return the vertices of the mesh so that their positions can be changed in place.
    /// After modifying the vertices you must call refitBVH for the refitting to actually take place.
    /// This function maintains the old mesh topology (triangle indices).
    ///
    /// inplace vertex coordinates for each existing mesh vertex.
    ///
    /// It is recommended to use this feature for scene queries only.
    ///
    /// Size of array returned is equal to the number returned by getNbVertices().
    ///
    /// This function operates on cooked vertex indices.
    ///
    /// This means the index mapping and vertex count can be different from what was provided as an input to the cooking routine.
    ///
    /// To achieve unchanged 1-to-1 index mapping with orignal mesh data (before cooking) please use the following cooking flags:
    ///
    /// eWELD_VERTICES = 0, eDISABLE_CLEAN_MESH = 1.
    ///
    /// It is also recommended to make sure that a call to validateTriangleMesh returns true if mesh cleaning is disabled.
physx_PxVec3*      physx_PxTriangleMesh_getVerticesForModification_mut(physx_PxTriangleMesh* self_);

    /// Refits BVH for mesh vertices.
    ///
    /// This function will refit the mesh BVH to correctly enclose the new positions updated by getVerticesForModification.
    /// Mesh BVH will not be reoptimized by this function so significantly different new positions will cause significantly reduced performance.
    ///
    /// New bounds for the entire mesh.
    ///
    /// For PxMeshMidPhase::eBVH34 trees the refit operation is only available on non-quantized trees (see PxBVH34MidphaseDesc::quantized)
    ///
    /// PhysX does not keep a mapping from the mesh to mesh shapes that reference it.
    ///
    /// Call PxShape::setGeometry on each shape which references the mesh, to ensure that internal data structures are updated to reflect the new geometry.
    ///
    /// PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
    ///
    /// It is also recommended to make sure that a call to validateTriangleMesh returns true if mesh cleaning is disabled.
    ///
    /// Active edges information will be lost during refit, the rigid body mesh contact generation might not perform as expected.
physx_PxBounds3      physx_PxTriangleMesh_refitBVH_mut(physx_PxTriangleMesh* self_);

    /// Returns the number of triangles.
    ///
    /// number of triangles
uint32_t      physx_PxTriangleMesh_getNbTriangles(physx_PxTriangleMesh const* self_);

    /// Returns the triangle indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of triangles in the mesh.
    /// Call getTriangleMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of triangles * 3.
    ///
    /// array of triangles
void const*      physx_PxTriangleMesh_getTriangles(physx_PxTriangleMesh const* self_);

    /// Reads the PxTriangleMesh flags.
    ///
    /// See the list of flags [`PxTriangleMeshFlag`]
    ///
    /// The values of the PxTriangleMesh flags.
uint8_t      physx_PxTriangleMesh_getTriangleMeshFlags(physx_PxTriangleMesh const* self_);

    /// Returns the triangle remapping table.
    ///
    /// The triangles are internally sorted according to various criteria. Hence the internal triangle order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTriangleIndex ] = originalTriangleIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
uint32_t const*      physx_PxTriangleMesh_getTrianglesRemap(physx_PxTriangleMesh const* self_);

    /// Decrements the reference count of a triangle mesh and releases it if the new reference count is zero.
void      physx_PxTriangleMesh_release_mut(physx_PxTriangleMesh* self_);

    /// Returns material table index of given triangle
    ///
    /// This function takes a post cooking triangle index.
    ///
    /// Material table index, or 0xffff if no per-triangle materials are used
uint16_t      physx_PxTriangleMesh_getTriangleMaterialIndex(physx_PxTriangleMesh const* self_, uint32_t triangleIndex);

    /// Returns the local-space (vertex space) AABB from the triangle mesh.
    ///
    /// local-space bounds
physx_PxBounds3      physx_PxTriangleMesh_getLocalBounds(physx_PxTriangleMesh const* self_);

    /// Returns the local-space Signed Distance Field for this mesh if it has one.
    ///
    /// local-space SDF.
float const*      physx_PxTriangleMesh_getSDF(physx_PxTriangleMesh const* self_);

    /// Returns the resolution of the local-space dense SDF.
void      physx_PxTriangleMesh_getSDFDimensions(physx_PxTriangleMesh const* self_, uint32_t* numX, uint32_t* numY, uint32_t* numZ);

    /// Sets whether this mesh should be preferred for SDF projection.
    ///
    /// By default, meshes are flagged as preferring projection and the decisions on which mesh to project is based on the triangle and vertex
    /// count. The model with the fewer triangles is projected onto the SDF of the more detailed mesh.
    /// If one of the meshes is set to prefer SDF projection (default) and the other is set to not prefer SDF projection, model flagged as
    /// preferring SDF projection will be projected onto the model flagged as not preferring, regardless of the detail of the respective meshes.
    /// Where both models are flagged as preferring no projection, the less detailed model will be projected as before.
void      physx_PxTriangleMesh_setPreferSDFProjection_mut(physx_PxTriangleMesh* self_, bool preferProjection);

    /// Returns whether this mesh prefers SDF projection.
    ///
    /// whether this mesh prefers SDF projection.
bool      physx_PxTriangleMesh_getPreferSDFProjection(physx_PxTriangleMesh const* self_);

    /// Returns the mass properties of the mesh assuming unit density.
    ///
    /// The following relationship holds between mass and volume:
    ///
    /// mass = volume * density
    ///
    /// The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.
    ///
    /// Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
    /// localInertia of the unit density mesh by d.
void      physx_PxTriangleMesh_getMassInformation(physx_PxTriangleMesh const* self_, float* mass, physx_PxMat33* localInertia, physx_PxVec3* localCenterOfMass);

    /// Constructor
physx_PxTetrahedron*      physx_PxTetrahedron_new_alloc();

    /// Constructor
physx_PxTetrahedron*      physx_PxTetrahedron_new_alloc_1(physx_PxVec3 const* p0, physx_PxVec3 const* p1, physx_PxVec3 const* p2, physx_PxVec3 const* p3);

    /// Destructor
void      physx_PxTetrahedron_delete(physx_PxTetrahedron* self_);

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
void      physx_PxSoftBodyAuxData_release_mut(physx_PxSoftBodyAuxData* self_);

    /// Returns the number of vertices.
    ///
    /// number of vertices
uint32_t      physx_PxTetrahedronMesh_getNbVertices(physx_PxTetrahedronMesh const* self_);

    /// Returns the vertices
    ///
    /// array of vertices
physx_PxVec3 const*      physx_PxTetrahedronMesh_getVertices(physx_PxTetrahedronMesh const* self_);

    /// Returns the number of tetrahedrons.
    ///
    /// number of tetrahedrons
uint32_t      physx_PxTetrahedronMesh_getNbTetrahedrons(physx_PxTetrahedronMesh const* self_);

    /// Returns the tetrahedron indices.
    ///
    /// The indices can be 16 or 32bit depending on the number of tetrahedrons in the mesh.
    /// Call getTetrahedronMeshFlags() to know if the indices are 16 or 32 bits.
    ///
    /// The number of indices is the number of tetrahedrons * 4.
    ///
    /// array of tetrahedrons
void const*      physx_PxTetrahedronMesh_getTetrahedrons(physx_PxTetrahedronMesh const* self_);

    /// Reads the PxTetrahedronMesh flags.
    ///
    /// See the list of flags [`PxTetrahedronMeshFlags`]
    ///
    /// The values of the PxTetrahedronMesh flags.
uint8_t      physx_PxTetrahedronMesh_getTetrahedronMeshFlags(physx_PxTetrahedronMesh const* self_);

    /// Returns the tetrahedra remapping table.
    ///
    /// The tetrahedra are internally sorted according to various criteria. Hence the internal tetrahedron order
    /// does not always match the original (user-defined) order. The remapping table helps finding the old
    /// indices knowing the new ones:
    ///
    /// remapTable[ internalTetrahedronIndex ] = originalTetrahedronIndex
    ///
    /// the remapping table (or NULL if 'PxCookingParams::suppressTriangleMeshRemapTable' has been used)
uint32_t const*      physx_PxTetrahedronMesh_getTetrahedraRemap(physx_PxTetrahedronMesh const* self_);

    /// Returns the local-space (vertex space) AABB from the tetrahedron mesh.
    ///
    /// local-space bounds
physx_PxBounds3      physx_PxTetrahedronMesh_getLocalBounds(physx_PxTetrahedronMesh const* self_);

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
void      physx_PxTetrahedronMesh_release_mut(physx_PxTetrahedronMesh* self_);

    /// Const accecssor to the softbody's collision mesh.
physx_PxTetrahedronMesh const*      physx_PxSoftBodyMesh_getCollisionMesh(physx_PxSoftBodyMesh const* self_);

    /// Accecssor to the softbody's collision mesh.
physx_PxTetrahedronMesh*      physx_PxSoftBodyMesh_getCollisionMesh_mut(physx_PxSoftBodyMesh* self_);

    /// Const accessor to the softbody's simulation mesh.
physx_PxTetrahedronMesh const*      physx_PxSoftBodyMesh_getSimulationMesh(physx_PxSoftBodyMesh const* self_);

    /// Accecssor to the softbody's simulation mesh.
physx_PxTetrahedronMesh*      physx_PxSoftBodyMesh_getSimulationMesh_mut(physx_PxSoftBodyMesh* self_);

    /// Const accessor to the softbodies simulation state.
physx_PxSoftBodyAuxData const*      physx_PxSoftBodyMesh_getSoftBodyAuxData(physx_PxSoftBodyMesh const* self_);

    /// Accessor to the softbody's auxilary data like mass and rest pose information
physx_PxSoftBodyAuxData*      physx_PxSoftBodyMesh_getSoftBodyAuxData_mut(physx_PxSoftBodyMesh* self_);

    /// Decrements the reference count of a tetrahedron mesh and releases it if the new reference count is zero.
void      physx_PxSoftBodyMesh_release_mut(physx_PxSoftBodyMesh* self_);

void      physx_PxCollisionMeshMappingData_release_mut(physx_PxCollisionMeshMappingData* self_);

physx_PxTetrahedronMeshData const*      physx_PxCollisionTetrahedronMeshData_getMesh(physx_PxCollisionTetrahedronMeshData const* self_);

physx_PxTetrahedronMeshData*      physx_PxCollisionTetrahedronMeshData_getMesh_mut(physx_PxCollisionTetrahedronMeshData* self_);

physx_PxSoftBodyCollisionData const*      physx_PxCollisionTetrahedronMeshData_getData(physx_PxCollisionTetrahedronMeshData const* self_);

physx_PxSoftBodyCollisionData*      physx_PxCollisionTetrahedronMeshData_getData_mut(physx_PxCollisionTetrahedronMeshData* self_);

void      physx_PxCollisionTetrahedronMeshData_release_mut(physx_PxCollisionTetrahedronMeshData* self_);

physx_PxTetrahedronMeshData*      physx_PxSimulationTetrahedronMeshData_getMesh_mut(physx_PxSimulationTetrahedronMeshData* self_);

physx_PxSoftBodySimulationData*      physx_PxSimulationTetrahedronMeshData_getData_mut(physx_PxSimulationTetrahedronMeshData* self_);

void      physx_PxSimulationTetrahedronMeshData_release_mut(physx_PxSimulationTetrahedronMeshData* self_);

    /// Deletes the actor.
    ///
    /// Do not keep a reference to the deleted instance.
    ///
    /// If the actor belongs to a [`PxAggregate`] object, it is automatically removed from the aggregate.
void      physx_PxActor_release_mut(physx_PxActor* self_);

    /// Retrieves the type of actor.
    ///
    /// The actor type of the actor.
int32_t      physx_PxActor_getType(physx_PxActor const* self_);

    /// Retrieves the scene which this actor belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
physx_PxScene*      physx_PxActor_getScene(physx_PxActor const* self_);

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
    ///
    /// Default:
    /// NULL
void      physx_PxActor_setName_mut(physx_PxActor* self_, char const* name);

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
char const*      physx_PxActor_getName(physx_PxActor const* self_);

    /// Retrieves the axis aligned bounding box enclosing the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The actor's bounding box.
physx_PxBounds3      physx_PxActor_getWorldBounds(physx_PxActor const* self_, float inflation);

    /// Raises or clears a particular actor flag.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxActor_setActorFlag_mut(physx_PxActor* self_, int32_t flag, bool value);

    /// Sets the actor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
void      physx_PxActor_setActorFlags_mut(physx_PxActor* self_, uint8_t inFlags);

    /// Reads the PxActor flags.
    ///
    /// See the list of flags [`PxActorFlag`]
    ///
    /// The values of the PxActor flags.
uint8_t      physx_PxActor_getActorFlags(physx_PxActor const* self_);

    /// Assigns dynamic actors a dominance group identifier.
    ///
    /// PxDominanceGroup is a 5 bit group identifier (legal range from 0 to 31).
    ///
    /// The PxScene::setDominanceGroupPair() lets you set certain behaviors for pairs of dominance groups.
    /// By default every dynamic actor is created in group 0.
    ///
    /// Default:
    /// 0
    ///
    /// Sleeping:
    /// Changing the dominance group does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxActor_setDominanceGroup_mut(physx_PxActor* self_, uint8_t dominanceGroup);

    /// Retrieves the value set with setDominanceGroup().
    ///
    /// The dominance group of this actor.
uint8_t      physx_PxActor_getDominanceGroup(physx_PxActor const* self_);

    /// Sets the owner client of an actor.
    ///
    /// This cannot be done once the actor has been placed into a scene.
    ///
    /// Default:
    /// PX_DEFAULT_CLIENT
void      physx_PxActor_setOwnerClient_mut(physx_PxActor* self_, uint8_t inClient);

    /// Returns the owner client that was specified at creation time.
    ///
    /// This value cannot be changed once the object is placed into the scene.
uint8_t      physx_PxActor_getOwnerClient(physx_PxActor const* self_);

    /// Retrieves the aggregate the actor might be a part of.
    ///
    /// The aggregate the actor is a part of, or NULL if the actor does not belong to an aggregate.
physx_PxAggregate*      physx_PxActor_getAggregate(physx_PxActor const* self_);

uint32_t      physx_phys_PxGetAggregateFilterHint(int32_t type_, bool enableSelfCollision);

uint32_t      physx_phys_PxGetAggregateSelfCollisionBit(uint32_t hint);

int32_t      physx_phys_PxGetAggregateType(uint32_t hint);

    /// Deletes the aggregate object.
    ///
    /// Deleting the PxAggregate object does not delete the aggregated actors. If the PxAggregate object
    /// belongs to a scene, the aggregated actors are automatically re-inserted in that scene. If you intend
    /// to delete both the PxAggregate and its actors, it is best to release the actors first, then release
    /// the PxAggregate when it is empty.
void      physx_PxAggregate_release_mut(physx_PxAggregate* self_);

    /// Adds an actor to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached, or if the incoming actor already belongs
    /// to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an actor to the aggregate also adds the actor to that scene.
    ///
    /// If the actor already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the actor from the scene first, before adding it to the aggregate.
    ///
    /// When a BVH is provided the actor shapes are grouped together.
    /// The scene query pruning structure inside PhysX SDK will store/update one
    /// bound per actor. The scene queries against such an actor will query actor
    /// bounds and then make a local space query against the provided BVH, which is in actor's local space.
bool      physx_PxAggregate_addActor_mut(physx_PxAggregate* self_, physx_PxActor* actor, physx_PxBVH const* bvh);

    /// Removes an actor from the aggregate object.
    ///
    /// A warning is output if the incoming actor does not belong to the aggregate. Otherwise the actor is
    /// removed from the aggregate. If the aggregate belongs to a scene, the actor is reinserted in that
    /// scene. If you intend to delete the actor, it is best to call [`PxActor::release`]() directly. That way
    /// the actor will be automatically removed from its aggregate (if any) and not reinserted in a scene.
bool      physx_PxAggregate_removeActor_mut(physx_PxAggregate* self_, physx_PxActor* actor);

    /// Adds an articulation to the aggregate object.
    ///
    /// A warning is output if the total number of actors is reached (every articulation link counts as an actor),
    /// or if the incoming articulation already belongs to an aggregate.
    ///
    /// If the aggregate belongs to a scene, adding an articulation to the aggregate also adds the articulation to that scene.
    ///
    /// If the articulation already belongs to a scene, a warning is output and the call is ignored. You need to remove
    /// the articulation from the scene first, before adding it to the aggregate.
bool      physx_PxAggregate_addArticulation_mut(physx_PxAggregate* self_, physx_PxArticulationReducedCoordinate* articulation);

    /// Removes an articulation from the aggregate object.
    ///
    /// A warning is output if the incoming articulation does not belong to the aggregate. Otherwise the articulation is
    /// removed from the aggregate. If the aggregate belongs to a scene, the articulation is reinserted in that
    /// scene. If you intend to delete the articulation, it is best to call [`PxArticulationReducedCoordinate::release`]() directly. That way
    /// the articulation will be automatically removed from its aggregate (if any) and not reinserted in a scene.
bool      physx_PxAggregate_removeArticulation_mut(physx_PxAggregate* self_, physx_PxArticulationReducedCoordinate* articulation);

    /// Returns the number of actors contained in the aggregate.
    ///
    /// You can use [`getActors`]() to retrieve the actor pointers.
    ///
    /// Number of actors contained in the aggregate.
uint32_t      physx_PxAggregate_getNbActors(physx_PxAggregate const* self_);

    /// Retrieves max amount of shapes that can be contained in the aggregate.
    ///
    /// Max shape size.
uint32_t      physx_PxAggregate_getMaxNbShapes(physx_PxAggregate const* self_);

    /// Retrieve all actors contained in the aggregate.
    ///
    /// You can retrieve the number of actor pointers by calling [`getNbActors`]()
    ///
    /// Number of actor pointers written to the buffer.
uint32_t      physx_PxAggregate_getActors(physx_PxAggregate const* self_, physx_PxActor** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Retrieves the scene which this aggregate belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
physx_PxScene*      physx_PxAggregate_getScene_mut(physx_PxAggregate* self_);

    /// Retrieves aggregate's self-collision flag.
    ///
    /// self-collision flag
bool      physx_PxAggregate_getSelfCollision(physx_PxAggregate const* self_);

char const*      physx_PxAggregate_getConcreteTypeName(physx_PxAggregate const* self_);

physx_PxConstraintInvMassScale      physx_PxConstraintInvMassScale_new();

physx_PxConstraintInvMassScale      physx_PxConstraintInvMassScale_new_1(float lin0, float ang0, float lin1, float ang1);

    /// Visualize joint frames
void      physx_PxConstraintVisualizer_visualizeJointFrames_mut(physx_PxConstraintVisualizer* self_, physx_PxTransform const* parent, physx_PxTransform const* child);

    /// Visualize joint linear limit
void      physx_PxConstraintVisualizer_visualizeLinearLimit_mut(physx_PxConstraintVisualizer* self_, physx_PxTransform const* t0, physx_PxTransform const* t1, float value, bool active);

    /// Visualize joint angular limit
void      physx_PxConstraintVisualizer_visualizeAngularLimit_mut(physx_PxConstraintVisualizer* self_, physx_PxTransform const* t0, float lower, float upper, bool active);

    /// Visualize limit cone
void      physx_PxConstraintVisualizer_visualizeLimitCone_mut(physx_PxConstraintVisualizer* self_, physx_PxTransform const* t, float tanQSwingY, float tanQSwingZ, bool active);

    /// Visualize joint double cone
void      physx_PxConstraintVisualizer_visualizeDoubleCone_mut(physx_PxConstraintVisualizer* self_, physx_PxTransform const* t, float angle, bool active);

    /// Visualize line
void      physx_PxConstraintVisualizer_visualizeLine_mut(physx_PxConstraintVisualizer* self_, physx_PxVec3 const* p0, physx_PxVec3 const* p1, uint32_t color);

    /// Pre-simulation data preparation
    /// when the constraint is marked dirty, this function is called at the start of the simulation
    /// step for the SDK to copy the constraint data block.
void*      physx_PxConstraintConnector_prepareData_mut(physx_PxConstraintConnector* self_);

    /// Constraint release callback
    ///
    /// When the SDK deletes a PxConstraint object this function is called by the SDK. In general
    /// custom constraints should not be deleted directly by applications: rather, the constraint
    /// should respond to a release() request by calling PxConstraint::release(), then wait for
    /// this call to release its own resources.
    ///
    /// This function is also called when a PxConstraint object is deleted on cleanup due to
    /// destruction of the PxPhysics object.
void      physx_PxConstraintConnector_onConstraintRelease_mut(physx_PxConstraintConnector* self_);

    /// Center-of-mass shift callback
    ///
    /// This function is called by the SDK when the CoM of one of the actors is moved. Since the
    /// API specifies constraint positions relative to actors, and the constraint shader functions
    /// are supplied with coordinates relative to bodies, some synchronization is usually required
    /// when the application moves an object's center of mass.
void      physx_PxConstraintConnector_onComShift_mut(physx_PxConstraintConnector* self_, uint32_t actor);

    /// Origin shift callback
    ///
    /// This function is called by the SDK when the scene origin gets shifted and allows to adjust
    /// custom data which contains world space transforms.
    ///
    /// If the adjustments affect constraint shader data, it is necessary to call PxConstraint::markDirty()
    /// to make sure that the data gets synced at the beginning of the next simulation step.
void      physx_PxConstraintConnector_onOriginShift_mut(physx_PxConstraintConnector* self_, physx_PxVec3 const* shift);

    /// Obtain a reference to a PxBase interface if the constraint has one.
    ///
    /// If the constraint does not implement the PxBase interface, it should return NULL.
physx_PxBase*      physx_PxConstraintConnector_getSerializable_mut(physx_PxConstraintConnector* self_);

    /// Obtain the pointer to the constraint's constant data
void const*      physx_PxConstraintConnector_getConstantBlock(physx_PxConstraintConnector const* self_);

    /// Let the connector know it has been connected to a constraint.
void      physx_PxConstraintConnector_connectToConstraint_mut(physx_PxConstraintConnector* self_, physx_PxConstraint* anon_param0);

    /// virtual destructor
void      physx_PxConstraintConnector_delete(physx_PxConstraintConnector* self_);

physx_PxSolverBody      physx_PxSolverBody_new();

float      physx_PxSolverBodyData_projectVelocity(physx_PxSolverBodyData const* self_, physx_PxVec3 const* lin, physx_PxVec3 const* ang);

void      physx_PxSolverConstraintPrepDesc_delete(physx_PxSolverConstraintPrepDesc* self_);

    /// Allocates constraint data. It is the application's responsibility to release this memory after PxSolveConstraints has completed.
    ///
    /// The allocated memory. This address must be 16-byte aligned.
uint8_t*      physx_PxConstraintAllocator_reserveConstraintData_mut(physx_PxConstraintAllocator* self_, uint32_t byteSize);

    /// Allocates friction data. Friction data can be retained by the application for a given pair and provided as an input to PxSolverContactDesc to improve simulation stability.
    /// It is the application's responsibility to release this memory. If this memory is released, the application should ensure it does not pass pointers to this memory to PxSolverContactDesc.
    ///
    /// The allocated memory. This address must be 4-byte aligned.
uint8_t*      physx_PxConstraintAllocator_reserveFrictionData_mut(physx_PxConstraintAllocator* self_, uint32_t byteSize);

void      physx_PxConstraintAllocator_delete(physx_PxConstraintAllocator* self_);

physx_PxArticulationLimit      physx_PxArticulationLimit_new();

physx_PxArticulationLimit      physx_PxArticulationLimit_new_1(float low_, float high_);

physx_PxArticulationDrive      physx_PxArticulationDrive_new();

physx_PxArticulationDrive      physx_PxArticulationDrive_new_1(float stiffness_, float damping_, float maxForce_, int32_t driveType_);

float      physx_PxTGSSolverBodyVel_projectVelocity(physx_PxTGSSolverBodyVel const* self_, physx_PxVec3 const* lin, physx_PxVec3 const* ang);

float      physx_PxTGSSolverBodyData_projectVelocity(physx_PxTGSSolverBodyData const* self_, physx_PxVec3 const* linear, physx_PxVec3 const* angular);

void      physx_PxTGSSolverConstraintPrepDesc_delete(physx_PxTGSSolverConstraintPrepDesc* self_);

    /// Sets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
void      physx_PxArticulationAttachment_setRestLength_mut(physx_PxArticulationAttachment* self_, float restLength);

    /// Gets the spring rest length for the sub-tendon from the root to this leaf attachment.
    ///
    /// The rest length.
float      physx_PxArticulationAttachment_getRestLength(physx_PxArticulationAttachment const* self_);

    /// Sets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Setting this on non-leaf attachments has no effect.
void      physx_PxArticulationAttachment_setLimitParameters_mut(physx_PxArticulationAttachment* self_, physx_PxArticulationTendonLimit const* parameters);

    /// Gets the low and high limit on the length of the sub-tendon from the root to this leaf attachment.
    ///
    /// Struct with the low and high limit.
physx_PxArticulationTendonLimit      physx_PxArticulationAttachment_getLimitParameters(physx_PxArticulationAttachment const* self_);

    /// Sets the attachment's relative offset in the link actor frame.
void      physx_PxArticulationAttachment_setRelativeOffset_mut(physx_PxArticulationAttachment* self_, physx_PxVec3 const* offset);

    /// Gets the attachment's relative offset in the link actor frame.
    ///
    /// The relative offset in the link actor frame.
physx_PxVec3      physx_PxArticulationAttachment_getRelativeOffset(physx_PxArticulationAttachment const* self_);

    /// Sets the attachment coefficient.
void      physx_PxArticulationAttachment_setCoefficient_mut(physx_PxArticulationAttachment* self_, float coefficient);

    /// Gets the attachment coefficient.
    ///
    /// The scale that the distance between this attachment and its parent is multiplied by when summing up the spatial tendon's length.
float      physx_PxArticulationAttachment_getCoefficient(physx_PxArticulationAttachment const* self_);

    /// Gets the articulation link.
    ///
    /// The articulation link that this attachment is attached to.
physx_PxArticulationLink*      physx_PxArticulationAttachment_getLink(physx_PxArticulationAttachment const* self_);

    /// Gets the parent attachment.
    ///
    /// The parent attachment.
physx_PxArticulationAttachment*      physx_PxArticulationAttachment_getParent(physx_PxArticulationAttachment const* self_);

    /// Indicates that this attachment is a leaf, and thus defines a sub-tendon from the root to this attachment.
    ///
    /// True: This attachment is a leaf and has zero children; False: Not a leaf.
bool      physx_PxArticulationAttachment_isLeaf(physx_PxArticulationAttachment const* self_);

    /// Gets the spatial tendon that the attachment is a part of.
    ///
    /// The tendon.
physx_PxArticulationSpatialTendon*      physx_PxArticulationAttachment_getTendon(physx_PxArticulationAttachment const* self_);

    /// Releases the attachment.
    ///
    /// Releasing the attachment is not allowed while the articulation is in a scene. In order to
    /// release the attachment, remove and then re-add the articulation to the scene.
void      physx_PxArticulationAttachment_release_mut(physx_PxArticulationAttachment* self_);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationAttachment_getConcreteTypeName(physx_PxArticulationAttachment const* self_);

    /// Sets the tendon joint coefficient.
    ///
    /// RecipCoefficient is commonly expected to be 1/coefficient, but it can be set to different values to tune behavior; for example, zero can be used to
    /// have a joint axis only participate in the length computation of the tendon, but not have any tendon force applied to it.
void      physx_PxArticulationTendonJoint_setCoefficient_mut(physx_PxArticulationTendonJoint* self_, int32_t axis, float coefficient, float recipCoefficient);

    /// Gets the tendon joint coefficient.
void      physx_PxArticulationTendonJoint_getCoefficient(physx_PxArticulationTendonJoint const* self_, int32_t* axis, float* coefficient, float* recipCoefficient);

    /// Gets the articulation link.
    ///
    /// The articulation link (and its incoming joint in particular) that this tendon joint is associated with.
physx_PxArticulationLink*      physx_PxArticulationTendonJoint_getLink(physx_PxArticulationTendonJoint const* self_);

    /// Gets the parent tendon joint.
    ///
    /// The parent tendon joint.
physx_PxArticulationTendonJoint*      physx_PxArticulationTendonJoint_getParent(physx_PxArticulationTendonJoint const* self_);

    /// Gets the tendon that the joint is a part of.
    ///
    /// The tendon.
physx_PxArticulationFixedTendon*      physx_PxArticulationTendonJoint_getTendon(physx_PxArticulationTendonJoint const* self_);

    /// Releases a tendon joint.
    ///
    /// Releasing a tendon joint is not allowed while the articulation is in a scene. In order to
    /// release the joint, remove and then re-add the articulation to the scene.
void      physx_PxArticulationTendonJoint_release_mut(physx_PxArticulationTendonJoint* self_);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationTendonJoint_getConcreteTypeName(physx_PxArticulationTendonJoint const* self_);

    /// Sets the spring stiffness term acting on the tendon length.
void      physx_PxArticulationTendon_setStiffness_mut(physx_PxArticulationTendon* self_, float stiffness);

    /// Gets the spring stiffness of the tendon.
    ///
    /// The spring stiffness.
float      physx_PxArticulationTendon_getStiffness(physx_PxArticulationTendon const* self_);

    /// Sets the damping term acting both on the tendon length and tendon-length limits.
void      physx_PxArticulationTendon_setDamping_mut(physx_PxArticulationTendon* self_, float damping);

    /// Gets the damping term acting both on the tendon length and tendon-length limits.
    ///
    /// The damping term.
float      physx_PxArticulationTendon_getDamping(physx_PxArticulationTendon const* self_);

    /// Sets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
void      physx_PxArticulationTendon_setLimitStiffness_mut(physx_PxArticulationTendon* self_, float stiffness);

    /// Gets the limit stiffness term acting on the tendon's length limits.
    ///
    /// For spatial tendons, this parameter applies to all its leaf attachments / sub-tendons.
    ///
    /// The limit stiffness term.
float      physx_PxArticulationTendon_getLimitStiffness(physx_PxArticulationTendon const* self_);

    /// Sets the length offset term for the tendon.
    ///
    /// An offset defines an amount to be added to the accumulated length computed for the tendon. It allows the
    /// application to actuate the tendon by shortening or lengthening it.
void      physx_PxArticulationTendon_setOffset_mut(physx_PxArticulationTendon* self_, float offset, bool autowake);

    /// Gets the length offset term for the tendon.
    ///
    /// The offset term.
float      physx_PxArticulationTendon_getOffset(physx_PxArticulationTendon const* self_);

    /// Gets the articulation that the tendon is a part of.
    ///
    /// The articulation.
physx_PxArticulationReducedCoordinate*      physx_PxArticulationTendon_getArticulation(physx_PxArticulationTendon const* self_);

    /// Releases a tendon to remove it from the articulation and free its associated memory.
    ///
    /// When an articulation is released, its attached tendons are automatically released.
    ///
    /// Releasing a tendon is not allowed while the articulation is in a scene. In order to
    /// release the tendon, remove and then re-add the articulation to the scene.
void      physx_PxArticulationTendon_release_mut(physx_PxArticulationTendon* self_);

    /// Creates an articulation attachment and adds it to the list of children in the parent attachment.
    ///
    /// Creating an attachment is not allowed while the articulation is in a scene. In order to
    /// add the attachment, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created attachment if creation was successful, otherwise a null pointer.
physx_PxArticulationAttachment*      physx_PxArticulationSpatialTendon_createAttachment_mut(physx_PxArticulationSpatialTendon* self_, physx_PxArticulationAttachment* parent, float coefficient, physx_PxVec3 relativeOffset, physx_PxArticulationLink* link);

    /// Fills a user-provided buffer of attachment pointers with the set of attachments.
    ///
    /// The number of attachments that were filled into the user buffer.
uint32_t      physx_PxArticulationSpatialTendon_getAttachments(physx_PxArticulationSpatialTendon const* self_, physx_PxArticulationAttachment** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of attachments in the tendon.
    ///
    /// The number of attachments.
uint32_t      physx_PxArticulationSpatialTendon_getNbAttachments(physx_PxArticulationSpatialTendon const* self_);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationSpatialTendon_getConcreteTypeName(physx_PxArticulationSpatialTendon const* self_);

    /// Creates an articulation tendon joint and adds it to the list of children in the parent tendon joint.
    ///
    /// Creating a tendon joint is not allowed while the articulation is in a scene. In order to
    /// add the joint, remove and then re-add the articulation to the scene.
    ///
    /// The newly-created tendon joint if creation was successful, otherwise a null pointer.
    ///
    /// - The axis motion must not be configured as PxArticulationMotion::eLOCKED.
    /// - The axis cannot be part of a fixed joint, i.e. joint configured as PxArticulationJointType::eFIX.
physx_PxArticulationTendonJoint*      physx_PxArticulationFixedTendon_createTendonJoint_mut(physx_PxArticulationFixedTendon* self_, physx_PxArticulationTendonJoint* parent, int32_t axis, float coefficient, float recipCoefficient, physx_PxArticulationLink* link);

    /// Fills a user-provided buffer of tendon-joint pointers with the set of tendon joints.
    ///
    /// The number of tendon joints filled into the user buffer.
uint32_t      physx_PxArticulationFixedTendon_getTendonJoints(physx_PxArticulationFixedTendon const* self_, physx_PxArticulationTendonJoint** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of tendon joints in the tendon.
    ///
    /// The number of tendon joints.
uint32_t      physx_PxArticulationFixedTendon_getNbTendonJoints(physx_PxArticulationFixedTendon const* self_);

    /// Sets the spring rest length of the tendon.
    ///
    /// The accumulated "length" of a fixed tendon is a linear combination of the joint axis positions that the tendon is
    /// associated with, scaled by the respective tendon joints' coefficients. As such, when the joint positions of all
    /// joints are zero, the accumulated length of a fixed tendon is zero.
    ///
    /// The spring of the tendon is not exerting any force on the articulation when the rest length is equal to the
    /// tendon's accumulated length plus the tendon offset.
void      physx_PxArticulationFixedTendon_setRestLength_mut(physx_PxArticulationFixedTendon* self_, float restLength);

    /// Gets the spring rest length of the tendon.
    ///
    /// The spring rest length of the tendon.
float      physx_PxArticulationFixedTendon_getRestLength(physx_PxArticulationFixedTendon const* self_);

    /// Sets the low and high limit on the length of the tendon.
    ///
    /// The limits, together with the damping and limit stiffness parameters, act on the accumulated length of the tendon.
void      physx_PxArticulationFixedTendon_setLimitParameters_mut(physx_PxArticulationFixedTendon* self_, physx_PxArticulationTendonLimit const* parameter);

    /// Gets the low and high limit on the length of the tendon.
    ///
    /// Struct with the low and high limit.
physx_PxArticulationTendonLimit      physx_PxArticulationFixedTendon_getLimitParameters(physx_PxArticulationFixedTendon const* self_);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationFixedTendon_getConcreteTypeName(physx_PxArticulationFixedTendon const* self_);

physx_PxArticulationCache      physx_PxArticulationCache_new();

    /// Releases an articulation cache.
void      physx_PxArticulationCache_release_mut(physx_PxArticulationCache* self_);

    /// Releases the sensor.
    ///
    /// Releasing a sensor is not allowed while the articulation is in a scene. In order to
    /// release a sensor, remove and then re-add the articulation to the scene.
void      physx_PxArticulationSensor_release_mut(physx_PxArticulationSensor* self_);

    /// Returns the spatial force in the local frame of the sensor.
    ///
    /// The spatial force.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
physx_PxSpatialForce      physx_PxArticulationSensor_getForces(physx_PxArticulationSensor const* self_);

    /// Returns the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// The transform link body frame -> sensor frame.
physx_PxTransform      physx_PxArticulationSensor_getRelativePose(physx_PxArticulationSensor const* self_);

    /// Sets the relative pose between this sensor and the body frame of the link that the sensor is attached to.
    ///
    /// The link body frame is at the center of mass and aligned with the principal axes of inertia, see PxRigidBody::getCMassLocalPose.
    ///
    /// Setting the sensor relative pose is not allowed while the articulation is in a scene. In order to
    /// set the pose, remove and then re-add the articulation to the scene.
void      physx_PxArticulationSensor_setRelativePose_mut(physx_PxArticulationSensor* self_, physx_PxTransform const* pose);

    /// Returns the link that this sensor is attached to.
    ///
    /// A pointer to the link.
physx_PxArticulationLink*      physx_PxArticulationSensor_getLink(physx_PxArticulationSensor const* self_);

    /// Returns the index of this sensor inside the articulation.
    ///
    /// The return value is only valid for sensors attached to articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
uint32_t      physx_PxArticulationSensor_getIndex(physx_PxArticulationSensor const* self_);

    /// Returns the articulation that this sensor is part of.
    ///
    /// A pointer to the articulation.
physx_PxArticulationReducedCoordinate*      physx_PxArticulationSensor_getArticulation(physx_PxArticulationSensor const* self_);

    /// Returns the sensor's flags.
    ///
    /// The current set of flags of the sensor.
uint8_t      physx_PxArticulationSensor_getFlags(physx_PxArticulationSensor const* self_);

    /// Sets a flag of the sensor.
    ///
    /// Setting the sensor flags is not allowed while the articulation is in a scene. In order to
    /// set the flags, remove and then re-add the articulation to the scene.
void      physx_PxArticulationSensor_setFlag_mut(physx_PxArticulationSensor* self_, int32_t flag, bool enabled);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationSensor_getConcreteTypeName(physx_PxArticulationSensor const* self_);

    /// Returns the scene which this articulation belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
physx_PxScene*      physx_PxArticulationReducedCoordinate_getScene(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the solver iteration counts for the articulation.
    ///
    /// The solver iteration count determines how accurately contacts, drives, and limits are resolved.
    /// Setting a higher position iteration count may therefore help in scenarios where the articulation
    /// is subject to many constraints; for example, a manipulator articulation with drives and joint limits
    /// that is grasping objects, or several such articulations interacting through contacts. Other situations
    /// where higher position iterations may improve simulation fidelity are: large mass ratios within the
    /// articulation or between the articulation and an object in contact with it; or strong drives in the
    /// articulation being used to manipulate a light object.
    ///
    /// If intersecting bodies are being depenetrated too violently, increase the number of velocity
    /// iterations. More velocity iterations will drive the relative exit velocity of the intersecting
    /// objects closer to the correct value given the restitution.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setSolverIterationCounts_mut(physx_PxArticulationReducedCoordinate* self_, uint32_t minPositionIters, uint32_t minVelocityIters);

    /// Returns the solver iteration counts.
void      physx_PxArticulationReducedCoordinate_getSolverIterationCounts(physx_PxArticulationReducedCoordinate const* self_, uint32_t* minPositionIters, uint32_t* minVelocityIters);

    /// Returns true if this articulation is sleeping.
    ///
    /// When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
    /// is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
    /// or a sleep-affecting property is changed by the user, the entire sleep mechanism should be transparent to the user.
    ///
    /// An articulation can only go to sleep if all links are ready for sleeping. An articulation is guaranteed to be awake
    /// if at least one of the following holds:
    ///
    /// The wake counter is positive (see [`setWakeCounter`]()).
    ///
    /// The linear or angular velocity of any link is non-zero.
    ///
    /// A non-zero force or torque has been applied to the articulation or any of its links.
    ///
    /// If an articulation is sleeping, the following state is guaranteed:
    ///
    /// The wake counter is zero.
    ///
    /// The linear and angular velocity of all links is zero.
    ///
    /// There is no force update pending.
    ///
    /// When an articulation gets inserted into a scene, it will be considered asleep if all the points above hold, else it will
    /// be treated as awake.
    ///
    /// If an articulation is asleep after the call to [`PxScene::fetchResults`]() returns, it is guaranteed that the poses of the
    /// links were not changed. You can use this information to avoid updating the transforms of associated objects.
    ///
    /// True if the articulation is sleeping.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
bool      physx_PxArticulationReducedCoordinate_isSleeping(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the mass-normalized energy threshold below which the articulation may go to sleep.
    ///
    /// The articulation will sleep if the energy of each link is below this threshold.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setSleepThreshold_mut(physx_PxArticulationReducedCoordinate* self_, float threshold);

    /// Returns the mass-normalized energy below which the articulation may go to sleep.
    ///
    /// The energy threshold for sleeping.
float      physx_PxArticulationReducedCoordinate_getSleepThreshold(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the mass-normalized kinetic energy threshold below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 0.01 * PxTolerancesScale::speed * PxTolerancesScale::speed
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setStabilizationThreshold_mut(physx_PxArticulationReducedCoordinate* self_, float threshold);

    /// Returns the mass-normalized kinetic energy below which the articulation may participate in stabilization.
    ///
    /// Articulations whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
float      physx_PxArticulationReducedCoordinate_getStabilizationThreshold(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the wake counter for the articulation in seconds.
    ///
    /// - The wake counter value determines the minimum amount of time until the articulation can be put to sleep.
    /// - An articulation will not be put to sleep if the energy is above the specified threshold (see [`setSleepThreshold`]())
    /// or if other awake objects are touching it.
    /// - Passing in a positive value will wake up the articulation automatically.
    ///
    /// Default:
    /// 0.4s (which corresponds to 20 frames for a time step of 0.02s)
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
void      physx_PxArticulationReducedCoordinate_setWakeCounter_mut(physx_PxArticulationReducedCoordinate* self_, float wakeCounterValue);

    /// Returns the wake counter of the articulation in seconds.
    ///
    /// The wake counter of the articulation in seconds.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
float      physx_PxArticulationReducedCoordinate_getWakeCounter(physx_PxArticulationReducedCoordinate const* self_);

    /// Wakes up the articulation if it is sleeping.
    ///
    /// - The articulation will get woken up and might cause other touching objects to wake up as well during the next simulation step.
    /// - This will set the wake counter of the articulation to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation,
    /// except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
void      physx_PxArticulationReducedCoordinate_wakeUp_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Forces the articulation to sleep.
    ///
    /// - The articulation will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    /// - This will set any applied force, the velocity, and the wake counter of all bodies in the articulation to zero.
    ///
    /// This call may not be made during simulation, and may only be made on articulations that are in a scene.
void      physx_PxArticulationReducedCoordinate_putToSleep_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Sets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// - The limit acts on the linear velocity of the entire articulation. The velocity is calculated from the total momentum
    /// and the spatial inertia of the articulation.
    /// - The limit only applies to floating-base articulations.
    /// - A benefit of the COM velocity limit is that it is evenly applied to the whole articulation, which results in fewer visual
    /// artifacts compared to link rigid-body damping or joint-velocity limits. However, these per-link or per-degree-of-freedom
    /// limits may still help avoid numerical issues.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setMaxCOMLinearVelocity_mut(physx_PxArticulationReducedCoordinate* self_, float maxLinearVelocity);

    /// Gets the limit on the magnitude of the linear velocity of the articulation's center of mass.
    ///
    /// The maximal linear velocity magnitude.
float      physx_PxArticulationReducedCoordinate_getMaxCOMLinearVelocity(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// - The limit acts on the angular velocity of the entire articulation. The velocity is calculated from the total momentum
    /// and the spatial inertia of the articulation.
    /// - The limit only applies to floating-base articulations.
    /// - A benefit of the COM velocity limit is that it is evenly applied to the whole articulation, which results in fewer visual
    /// artifacts compared to link rigid-body damping or joint-velocity limits. However, these per-link or per-degree-of-freedom
    /// limits may still help avoid numerical issues.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setMaxCOMAngularVelocity_mut(physx_PxArticulationReducedCoordinate* self_, float maxAngularVelocity);

    /// Gets the limit on the magnitude of the angular velocity at the articulation's center of mass.
    ///
    /// The maximal angular velocity magnitude.
float      physx_PxArticulationReducedCoordinate_getMaxCOMAngularVelocity(physx_PxArticulationReducedCoordinate const* self_);

    /// Adds a link to the articulation with default attribute values.
    ///
    /// The new link, or NULL if the link cannot be created.
    ///
    /// Creating a link is not allowed while the articulation is in a scene. In order to add a link,
    /// remove and then re-add the articulation to the scene.
physx_PxArticulationLink*      physx_PxArticulationReducedCoordinate_createLink_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxArticulationLink* parent, physx_PxTransform const* pose);

    /// Releases the articulation, and all its links and corresponding joints.
    ///
    /// Attached sensors and tendons are released automatically when the articulation is released.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_release_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Returns the number of links in the articulation.
    ///
    /// The number of links.
uint32_t      physx_PxArticulationReducedCoordinate_getNbLinks(physx_PxArticulationReducedCoordinate const* self_);

    /// Returns the set of links in the articulation in the order that they were added to the articulation using createLink.
    ///
    /// The number of links written into the buffer.
uint32_t      physx_PxArticulationReducedCoordinate_getLinks(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationLink** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of shapes in the articulation.
    ///
    /// The number of shapes.
uint32_t      physx_PxArticulationReducedCoordinate_getNbShapes(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets a name string for the articulation that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
void      physx_PxArticulationReducedCoordinate_setName_mut(physx_PxArticulationReducedCoordinate* self_, char const* name);

    /// Returns the name string set with setName().
    ///
    /// Name string associated with the articulation.
char const*      physx_PxArticulationReducedCoordinate_getName(physx_PxArticulationReducedCoordinate const* self_);

    /// Returns the axis-aligned bounding box enclosing the articulation.
    ///
    /// The articulation's bounding box.
    ///
    /// It is not allowed to use this method while the simulation is running, except in a split simulation
    /// during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
physx_PxBounds3      physx_PxArticulationReducedCoordinate_getWorldBounds(physx_PxArticulationReducedCoordinate const* self_, float inflation);

    /// Returns the aggregate the articulation might be a part of.
    ///
    /// The aggregate the articulation is a part of, or NULL if the articulation does not belong to an aggregate.
physx_PxAggregate*      physx_PxArticulationReducedCoordinate_getAggregate(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets flags on the articulation.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setArticulationFlags_mut(physx_PxArticulationReducedCoordinate* self_, uint8_t flags);

    /// Raises or clears a flag on the articulation.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setArticulationFlag_mut(physx_PxArticulationReducedCoordinate* self_, int32_t flag, bool value);

    /// Returns the articulation's flags.
    ///
    /// The flags.
uint8_t      physx_PxArticulationReducedCoordinate_getArticulationFlags(physx_PxArticulationReducedCoordinate const* self_);

    /// Returns the total number of joint degrees-of-freedom (DOFs) of the articulation.
    ///
    /// - The six DOFs of the base of a floating-base articulation are not included in this count.
    /// - Example: Both a fixed-base and a floating-base double-pendulum with two revolute joints will have getDofs() == 2.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of joint DOFs, or 0xFFFFFFFF if the articulation is not in a scene.
uint32_t      physx_PxArticulationReducedCoordinate_getDofs(physx_PxArticulationReducedCoordinate const* self_);

    /// Creates an articulation cache that can be used to read and write internal articulation data.
    ///
    /// - When the structure of the articulation changes (e.g. adding a link or sensor) after the cache was created,
    /// the cache needs to be released and recreated.
    /// - Free the memory allocated for the cache by calling the release() method on the cache.
    /// - Caches can only be created by articulations that are in a scene.
    ///
    /// The cache, or NULL if the articulation is not in a scene.
physx_PxArticulationCache*      physx_PxArticulationReducedCoordinate_createCache(physx_PxArticulationReducedCoordinate const* self_);

    /// Returns the size of the articulation cache in bytes.
    ///
    /// - The size does not include: the user-allocated memory for the coefficient matrix or lambda values;
    /// the scratch-related memory/members; and the cache version. See comment in [`PxArticulationCache`].
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The byte size of the cache, or 0xFFFFFFFF if the articulation is not in a scene.
uint32_t      physx_PxArticulationReducedCoordinate_getCacheDataSize(physx_PxArticulationReducedCoordinate const* self_);

    /// Zeroes all data in the articulation cache, except user-provided and scratch memory, and cache version.
    ///
    /// This call may only be made on articulations that are in a scene.
void      physx_PxArticulationReducedCoordinate_zeroCache(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Applies the data in the cache to the articulation.
    ///
    /// This call wakes the articulation if it is sleeping, and the autowake parameter is true (default) or:
    /// - a nonzero joint velocity is applied or
    /// - a nonzero joint force is applied or
    /// - a nonzero root velocity is applied
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_applyCache_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxArticulationCache* cache, uint32_t flags, bool autowake);

    /// Copies internal data of the articulation to the cache.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_copyInternalStateToCache(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache, uint32_t flags);

    /// Converts maximal-coordinate joint DOF data to reduced coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
void      physx_PxArticulationReducedCoordinate_packJointData(physx_PxArticulationReducedCoordinate const* self_, float const* maximum, float* reduced);

    /// Converts reduced-coordinate joint DOF data to maximal coordinates.
    ///
    /// - Indexing into the maximal joint DOF data is via the link's low-level index minus 1 (the root link is not included).
    /// - The reduced-coordinate data follows the cache indexing convention, see PxArticulationCache::jointVelocity.
    ///
    /// The articulation must be in a scene.
void      physx_PxArticulationReducedCoordinate_unpackJointData(physx_PxArticulationReducedCoordinate const* self_, float const* reduced, float* maximum);

    /// Prepares common articulation data based on articulation pose for inverse dynamics calculations.
    ///
    /// Usage:
    /// 1. Set articulation pose (joint positions and base transform) via articulation cache and applyCache().
    /// 1. Call commonInit.
    /// 1. Call inverse dynamics computation method.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_commonInit(physx_PxArticulationReducedCoordinate const* self_);

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// - Inputs - Articulation pose (joint positions + base transform).
    /// - Outputs - Joint forces to counteract gravity (in cache).
    ///
    /// - The joint forces returned are determined purely by gravity for the articulation in the current joint and base pose, and joints at rest;
    /// i.e. external forces, joint velocities, and joint accelerations are set to zero. Joint drives are also not considered in the computation.
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeGeneralizedGravityForce(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Computes the joint DOF forces required to counteract Coriolis and centrifugal forces for the given articulation state.
    ///
    /// - Inputs - Articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint forces to counteract Coriolis and centrifugal forces (in cache).
    ///
    /// - The joint forces returned are determined purely by the articulation's state; i.e. external forces, gravity, and joint accelerations are set to zero.
    /// Joint drives and potential damping terms, such as link angular or linear damping, or joint friction, are also not considered in the computation.
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeCoriolisAndCentrifugalForce(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Computes the joint DOF forces required to counteract external spatial forces applied to articulation links.
    ///
    /// - Inputs - External forces on links (in cache), articulation pose (joint positions + base transform).
    /// - Outputs - Joint forces to counteract the external forces (in cache).
    ///
    /// - Only the external spatial forces provided in the cache and the articulation pose are considered in the computation.
    /// - The external spatial forces are with respect to the links' centers of mass, and not the actor's origin.
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeGeneralizedExternalForce(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Computes the joint accelerations for the given articulation state and joint forces.
    ///
    /// - Inputs - Joint forces (in cache) and articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint accelerations (in cache).
    ///
    /// - The computation includes Coriolis terms and gravity. However, joint drives and potential damping terms are not considered in the computation
    /// (for example, linear link damping or joint friction).
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeJointAcceleration(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Computes the joint forces for the given articulation state and joint accelerations, not considering gravity.
    ///
    /// - Inputs - Joint accelerations (in cache) and articulation state (joint positions and velocities (in cache), and base transform and spatial velocity).
    /// - Outputs - Joint forces (in cache).
    ///
    /// - The computation includes Coriolis terms. However, joint drives and potential damping terms are not considered in the computation
    /// (for example, linear link damping or joint friction).
    /// - Prior to the computation, update/set the base spatial velocity with PxArticulationCache::rootLinkData and applyCache().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeJointForce(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Compute the dense Jacobian for the articulation in world space, including the DOFs of a potentially floating base.
    ///
    /// This computes the dense representation of an inherently sparse matrix. Multiplication with this matrix maps
    /// joint space velocities to world-space linear and angular (i.e. spatial) velocities of the centers of mass of the links.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeDenseJacobian(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache, uint32_t* nRows, uint32_t* nCols);

    /// Computes the coefficient matrix for contact forces.
    ///
    /// - The matrix dimension is getCoefficientMatrixSize() = getDofs() * getNbLoopJoints(), and the DOF (column) indexing follows the internal DOF order, see PxArticulationCache::jointVelocity.
    /// - Each column in the matrix is the joint forces effected by a contact based on impulse strength 1.
    /// - The user must allocate memory for PxArticulationCache::coefficientMatrix where the required size of the PxReal array is equal to getCoefficientMatrixSize().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeCoefficientMatrix(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Computes the lambda values when the test impulse is 1.
    ///
    /// - The user must allocate memory for PxArticulationCache::lambda where the required size of the PxReal array is equal to getNbLoopJoints().
    /// - commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// True if convergence was achieved within maxIter; False if convergence was not achieved or the operation failed otherwise.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
bool      physx_PxArticulationReducedCoordinate_computeLambda(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache, physx_PxArticulationCache* initialState, float const*const jointTorque, uint32_t maxIter);

    /// Compute the joint-space inertia matrix that maps joint accelerations to joint forces: forces = M * accelerations.
    ///
    /// - Inputs - Articulation pose (joint positions and base transform).
    /// - Outputs - Mass matrix (in cache).
    ///
    /// commonInit() must be called before the computation, and after setting the articulation pose via applyCache().
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_computeGeneralizedMassMatrix(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationCache* cache);

    /// Adds a loop joint to the articulation system for inverse dynamics.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_addLoopJoint_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxConstraint* joint);

    /// Removes a loop joint from the articulation for inverse dynamics.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_removeLoopJoint_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxConstraint* joint);

    /// Returns the number of loop joints in the articulation for inverse dynamics.
    ///
    /// The number of loop joints.
uint32_t      physx_PxArticulationReducedCoordinate_getNbLoopJoints(physx_PxArticulationReducedCoordinate const* self_);

    /// Returns the set of loop constraints (i.e. joints) in the articulation.
    ///
    /// The number of constraints written into the buffer.
uint32_t      physx_PxArticulationReducedCoordinate_getLoopJoints(physx_PxArticulationReducedCoordinate const* self_, physx_PxConstraint** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the required size of the coefficient matrix in the articulation.
    ///
    /// Size of the coefficient matrix (equal to getDofs() * getNbLoopJoints()).
    ///
    /// This call may only be made on articulations that are in a scene.
uint32_t      physx_PxArticulationReducedCoordinate_getCoefficientMatrixSize(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the root link transform (world to actor frame).
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link transform in a batch articulation state update.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_setRootGlobalPose_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxTransform const* pose, bool autowake);

    /// Returns the root link transform (world to actor frame).
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link transform in a batch query.
    ///
    /// The root link transform.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
physx_PxTransform      physx_PxArticulationReducedCoordinate_getRootGlobalPose(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the root link linear center-of-mass velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
void      physx_PxArticulationReducedCoordinate_setRootLinearVelocity_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxVec3 const* linearVelocity, bool autowake);

    /// Gets the root link center-of-mass linear velocity.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link center-of-mass linear velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
physx_PxVec3      physx_PxArticulationReducedCoordinate_getRootLinearVelocity(physx_PxArticulationReducedCoordinate const* self_);

    /// Sets the root link angular velocity.
    ///
    /// - For performance, prefer PxArticulationCache::rootLinkData to set the root link velocity in a batch articulation state update.
    /// - The articulation is woken up if the input velocity is nonzero (ignoring autowake) and the articulation is in a scene.
    /// - Use updateKinematic() after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call may not be made during simulation, except in a split simulation in-between [`PxScene::fetchCollision`] and #PxScene::advance.
void      physx_PxArticulationReducedCoordinate_setRootAngularVelocity_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxVec3 const* angularVelocity, bool autowake);

    /// Gets the root link angular velocity.
    ///
    /// For performance, prefer PxArticulationCache::rootLinkData to get the root link velocity in a batch query.
    ///
    /// The root link angular velocity.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
physx_PxVec3      physx_PxArticulationReducedCoordinate_getRootAngularVelocity(physx_PxArticulationReducedCoordinate const* self_);

    /// Returns the (classical) link acceleration in world space for the given low-level link index.
    ///
    /// - The returned acceleration is not a spatial, but a classical, i.e. body-fixed acceleration (https://en.wikipedia.org/wiki/Spatial_acceleration).
    /// - The (linear) acceleration is with respect to the link's center of mass and not the actor frame origin.
    ///
    /// The link's center-of-mass classical acceleration, or 0 if the call is made before the articulation participated in a first simulation step.
    ///
    /// This call may only be made on articulations that are in a scene, and it is not allowed to use this method while the simulation
    /// is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(), and in PxContactModifyCallback or in contact report callbacks.
physx_PxSpatialVelocity      physx_PxArticulationReducedCoordinate_getLinkAcceleration_mut(physx_PxArticulationReducedCoordinate* self_, uint32_t linkId);

    /// Returns the GPU articulation index.
    ///
    /// The GPU index, or 0xFFFFFFFF if the articulation is not in a scene or PxSceneFlag::eSUPPRESS_READBACK is not set.
uint32_t      physx_PxArticulationReducedCoordinate_getGpuArticulationIndex_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Creates a spatial tendon to attach to the articulation with default attribute values.
    ///
    /// The new spatial tendon.
    ///
    /// Creating a spatial tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
physx_PxArticulationSpatialTendon*      physx_PxArticulationReducedCoordinate_createSpatialTendon_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Creates a fixed tendon to attach to the articulation with default attribute values.
    ///
    /// The new fixed tendon.
    ///
    /// Creating a fixed tendon is not allowed while the articulation is in a scene. In order to
    /// add the tendon, remove and then re-add the articulation to the scene.
physx_PxArticulationFixedTendon*      physx_PxArticulationReducedCoordinate_createFixedTendon_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Creates a force sensor attached to a link of the articulation.
    ///
    /// The new sensor.
    ///
    /// Creating a sensor is not allowed while the articulation is in a scene. In order to
    /// add the sensor, remove and then re-add the articulation to the scene.
physx_PxArticulationSensor*      physx_PxArticulationReducedCoordinate_createSensor_mut(physx_PxArticulationReducedCoordinate* self_, physx_PxArticulationLink* link, physx_PxTransform const* relativePose);

    /// Returns the spatial tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
uint32_t      physx_PxArticulationReducedCoordinate_getSpatialTendons(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationSpatialTendon** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of spatial tendons in the articulation.
    ///
    /// The number of tendons.
uint32_t      physx_PxArticulationReducedCoordinate_getNbSpatialTendons_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Returns the fixed tendons attached to the articulation.
    ///
    /// The order of the tendons in the buffer is not necessarily identical to the order in which the tendons were added to the articulation.
    ///
    /// The number of tendons written into the buffer.
uint32_t      physx_PxArticulationReducedCoordinate_getFixedTendons(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationFixedTendon** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of fixed tendons in the articulation.
    ///
    /// The number of tendons.
uint32_t      physx_PxArticulationReducedCoordinate_getNbFixedTendons_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Returns the sensors attached to the articulation.
    ///
    /// The order of the sensors in the buffer is not necessarily identical to the order in which the sensors were added to the articulation.
    ///
    /// The number of sensors written into the buffer.
uint32_t      physx_PxArticulationReducedCoordinate_getSensors(physx_PxArticulationReducedCoordinate const* self_, physx_PxArticulationSensor** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of sensors in the articulation.
    ///
    /// The number of sensors.
uint32_t      physx_PxArticulationReducedCoordinate_getNbSensors_mut(physx_PxArticulationReducedCoordinate* self_);

    /// Update link velocities and/or positions in the articulation.
    ///
    /// For performance, prefer the PxArticulationCache API that performs batch articulation state updates.
    ///
    /// If the application updates the root state (position and velocity) or joint state via any combination of
    /// the non-cache API calls
    ///
    /// - setRootGlobalPose(), setRootLinearVelocity(), setRootAngularVelocity()
    /// - PxArticulationJointReducedCoordinate::setJointPosition(), PxArticulationJointReducedCoordinate::setJointVelocity()
    ///
    /// the application needs to call this method after the state setting in order to update the link states for
    /// the next simulation frame or querying.
    ///
    /// Use
    /// - PxArticulationKinematicFlag::ePOSITION after any changes to the articulation root or joint positions using non-cache API calls. Updates links' positions and velocities.
    /// - PxArticulationKinematicFlag::eVELOCITY after velocity-only changes to the articulation root or joints using non-cache API calls. Updates links' velocities only.
    ///
    /// This call may only be made on articulations that are in a scene, and may not be made during simulation.
void      physx_PxArticulationReducedCoordinate_updateKinematic_mut(physx_PxArticulationReducedCoordinate* self_, uint8_t flags);

    /// Gets the parent articulation link of this joint.
    ///
    /// The parent link.
physx_PxArticulationLink*      physx_PxArticulationJointReducedCoordinate_getParentArticulationLink(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Sets the joint pose in the parent link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setParentPose_mut(physx_PxArticulationJointReducedCoordinate* self_, physx_PxTransform const* pose);

    /// Gets the joint pose in the parent link actor frame.
    ///
    /// The joint pose.
physx_PxTransform      physx_PxArticulationJointReducedCoordinate_getParentPose(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Gets the child articulation link of this joint.
    ///
    /// The child link.
physx_PxArticulationLink*      physx_PxArticulationJointReducedCoordinate_getChildArticulationLink(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Sets the joint pose in the child link actor frame.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setChildPose_mut(physx_PxArticulationJointReducedCoordinate* self_, physx_PxTransform const* pose);

    /// Gets the joint pose in the child link actor frame.
    ///
    /// The joint pose.
physx_PxTransform      physx_PxArticulationJointReducedCoordinate_getChildPose(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Sets the joint type (e.g. revolute).
    ///
    /// Setting the joint type is not allowed while the articulation is in a scene.
    /// In order to set the joint type, remove and then re-add the articulation to the scene.
void      physx_PxArticulationJointReducedCoordinate_setJointType_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t jointType);

    /// Gets the joint type.
    ///
    /// The joint type.
int32_t      physx_PxArticulationJointReducedCoordinate_getJointType(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Sets the joint motion for a given axis.
    ///
    /// Setting the motion of joint axes is not allowed while the articulation is in a scene.
    /// In order to set the motion, remove and then re-add the articulation to the scene.
void      physx_PxArticulationJointReducedCoordinate_setMotion_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, int32_t motion);

    /// Returns the joint motion for the given axis.
    ///
    /// The joint motion of the given axis.
int32_t      physx_PxArticulationJointReducedCoordinate_getMotion(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Sets the joint limits for a given axis.
    ///
    /// - The motion of the corresponding axis should be set to PxArticulationMotion::eLIMITED in order for the limits to be enforced.
    /// - The lower limit should be strictly smaller than the higher limit. If the limits should be equal, use PxArticulationMotion::eLOCKED
    /// and an appropriate offset in the parent/child joint frames.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, limit.min and limit.max must both be in range [-Pi, Pi].
void      physx_PxArticulationJointReducedCoordinate_setLimitParams_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, physx_PxArticulationLimit const* limit);

    /// Returns the joint limits for a given axis.
    ///
    /// The joint limits.
physx_PxArticulationLimit      physx_PxArticulationJointReducedCoordinate_getLimitParams(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Configures a joint drive for the given axis.
    ///
    /// See PxArticulationDrive for parameter details; and the manual for further information, and the drives' implicit spring-damper (i.e. PD control) implementation in particular.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setDriveParams_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, physx_PxArticulationDrive const* drive);

    /// Gets the joint drive configuration for the given axis.
    ///
    /// The drive parameters.
physx_PxArticulationDrive      physx_PxArticulationJointReducedCoordinate_getDriveParams(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Sets the joint drive position target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) for a translational axis, or rad for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, target must be in range [-Pi, Pi].
    ///
    /// The target is specified in the parent frame of the joint. If Gp, Gc are the parent and child actor poses in the world frame and Lp, Lc are the parent and child joint frames expressed in the parent and child actor frames then the joint will drive the parent and child links to poses that obey Gp * Lp * J = Gc * Lc. For joints restricted to angular motion, J has the form PxTranfsorm(PxVec3(PxZero), PxExp(PxVec3(twistTarget, swing1Target, swing2Target))).  For joints restricted to linear motion, J has the form PxTransform(PxVec3(XTarget, YTarget, ZTarget), PxQuat(PxIdentity)).
    ///
    /// For spherical joints with more than 1 degree of freedom, the joint target angles taken together can collectively represent a rotation of greater than Pi around a vector. When this happens the rotation that matches the joint drive target is not the shortest path rotation.  The joint pose J that is the outcome after driving to the target pose will always be the equivalent of the shortest path rotation.
void      physx_PxArticulationJointReducedCoordinate_setDriveTarget_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, float target, bool autowake);

    /// Returns the joint drive position target for the given axis.
    ///
    /// The target position.
float      physx_PxArticulationJointReducedCoordinate_getDriveTarget(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Sets the joint drive velocity target for the given axis.
    ///
    /// The target units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setDriveVelocity_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, float targetVel, bool autowake);

    /// Returns the joint drive velocity target for the given axis.
    ///
    /// The target velocity.
float      physx_PxArticulationJointReducedCoordinate_getDriveVelocity(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Sets the joint armature for the given axis.
    ///
    /// - The armature is directly added to the joint-space spatial inertia of the corresponding axis.
    /// - The armature is in mass units for a prismatic (i.e. linear) joint, and in mass units * (scene linear units)^2 for a rotational joint.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setArmature_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, float armature);

    /// Gets the joint armature for the given axis.
    ///
    /// The armature set on the given axis.
float      physx_PxArticulationJointReducedCoordinate_getArmature(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Sets the joint friction coefficient, which applies to all joint axes.
    ///
    /// - The joint friction is unitless and relates the magnitude of the spatial force [F_trans, T_trans] transmitted from parent to child link to
    /// the maximal friction force F_resist that may be applied by the solver to resist joint motion, per axis; i.e. |F_resist|
    /// <
    /// = coefficient * (|F_trans| + |T_trans|),
    /// where F_resist may refer to a linear force or torque depending on the joint axis.
    /// - The simulated friction effect is therefore similar to static and Coulomb friction. In order to simulate dynamic joint friction, use a joint drive with
    /// zero stiffness and zero velocity target, and an appropriately dimensioned damping parameter.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setFrictionCoefficient_mut(physx_PxArticulationJointReducedCoordinate* self_, float coefficient);

    /// Gets the joint friction coefficient.
    ///
    /// The joint friction coefficient.
float      physx_PxArticulationJointReducedCoordinate_getFrictionCoefficient(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Sets the maximal joint velocity enforced for all axes.
    ///
    /// - The solver will apply appropriate joint-space impulses in order to enforce the per-axis joint-velocity limit.
    /// - The velocity units are linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setMaxJointVelocity_mut(physx_PxArticulationJointReducedCoordinate* self_, float maxJointV);

    /// Gets the maximal joint velocity enforced for all axes.
    ///
    /// The maximal per-axis joint velocity.
float      physx_PxArticulationJointReducedCoordinate_getMaxJointVelocity(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Sets the joint position for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointPosition to set joint positions in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
    ///
    /// For spherical joints, jointPos must be in range [-Pi, Pi].
    ///
    /// Joint position is specified in the parent frame of the joint. If Gp, Gc are the parent and child actor poses in the world frame and Lp, Lc are the parent and child joint frames expressed in the parent and child actor frames then the parent and child links will be given poses that obey Gp * Lp * J = Gc * Lc with J denoting the joint pose. For joints restricted to angular motion, J has the form PxTranfsorm(PxVec3(PxZero), PxExp(PxVec3(twistPos, swing1Pos, swing2Pos))).  For joints restricted to linear motion, J has the form PxTransform(PxVec3(xPos, yPos, zPos), PxQuat(PxIdentity)).
    ///
    /// For spherical joints with more than 1 degree of freedom, the input joint positions taken together can collectively represent a rotation of greater than Pi around a vector. When this happens the rotation that matches the joint positions is not the shortest path rotation.  The joint pose J that is the outcome of setting and applying the joint positions will always be the equivalent of the shortest path rotation.
void      physx_PxArticulationJointReducedCoordinate_setJointPosition_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, float jointPos);

    /// Gets the joint position for the given axis, i.e. joint degree of freedom (DOF).
    ///
    /// For performance, prefer PxArticulationCache::jointPosition to get joint positions in a batch query.
    ///
    /// The joint position in linear units (equivalent to scene units) for a translational axis, or radians for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
float      physx_PxArticulationJointReducedCoordinate_getJointPosition(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Sets the joint velocity for the given axis.
    ///
    /// - For performance, prefer PxArticulationCache::jointVelocity to set joint velocities in a batch articulation state update.
    /// - Use PxArticulationReducedCoordinate::updateKinematic after all state updates to the articulation via non-cache API such as this method,
    /// in order to update link states for the next simulation frame or querying.
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationJointReducedCoordinate_setJointVelocity_mut(physx_PxArticulationJointReducedCoordinate* self_, int32_t axis, float jointVel);

    /// Gets the joint velocity for the given axis.
    ///
    /// For performance, prefer PxArticulationCache::jointVelocity to get joint velocities in a batch query.
    ///
    /// The joint velocity in linear units (equivalent to scene units) per second for a translational axis, or radians per second for a rotational axis.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
float      physx_PxArticulationJointReducedCoordinate_getJointVelocity(physx_PxArticulationJointReducedCoordinate const* self_, int32_t axis);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationJointReducedCoordinate_getConcreteTypeName(physx_PxArticulationJointReducedCoordinate const* self_);

    /// Decrements the reference count of a shape and releases it if the new reference count is zero.
    ///
    /// Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape
    /// created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
    /// a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
    /// and so when the shape is detached it will also be destroyed.
void      physx_PxShape_release_mut(physx_PxShape* self_);

    /// Adjust the geometry of the shape.
    ///
    /// The type of the passed in geometry must match the geometry type of the shape.
    ///
    /// It is not allowed to change the geometry type of a shape.
    ///
    /// This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.
void      physx_PxShape_setGeometry_mut(physx_PxShape* self_, physx_PxGeometry const* geometry);

    /// Retrieve a reference to the shape's geometry.
    ///
    /// The returned reference has the same lifetime as the PxShape it comes from.
    ///
    /// Reference to internal PxGeometry object.
physx_PxGeometry const*      physx_PxShape_getGeometry(physx_PxShape const* self_);

    /// Retrieves the actor which this shape is associated with.
    ///
    /// The actor this shape is associated with, if it is an exclusive shape, else NULL
physx_PxRigidActor*      physx_PxShape_getActor(physx_PxShape const* self_);

    /// Sets the pose of the shape in actor space, i.e. relative to the actors to which they are attached.
    ///
    /// This transformation is identity by default.
    ///
    /// The local pose is an attribute of the shape, and so will apply to all actors to which the shape is attached.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Note:
    /// Does not automatically update the inertia properties of the owning actor (if applicable); use the
    /// PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]() to do this.
    ///
    /// Default:
    /// the identity transform
void      physx_PxShape_setLocalPose_mut(physx_PxShape* self_, physx_PxTransform const* pose);

    /// Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.
    ///
    /// This transformation is identity by default.
    ///
    /// Pose of shape relative to the actor's frame.
physx_PxTransform      physx_PxShape_getLocalPose(physx_PxShape const* self_);

    /// Sets the user definable collision filter data.
    ///
    /// Sleeping:
    /// Does wake up the actor if the filter data change causes a formerly suppressed
    /// collision pair to be enabled.
    ///
    /// Default:
    /// (0,0,0,0)
void      physx_PxShape_setSimulationFilterData_mut(physx_PxShape* self_, physx_PxFilterData const* data);

    /// Retrieves the shape's collision filter data.
physx_PxFilterData      physx_PxShape_getSimulationFilterData(physx_PxShape const* self_);

    /// Sets the user definable query filter data.
    ///
    /// Default:
    /// (0,0,0,0)
void      physx_PxShape_setQueryFilterData_mut(physx_PxShape* self_, physx_PxFilterData const* data);

    /// Retrieves the shape's Query filter data.
physx_PxFilterData      physx_PxShape_getQueryFilterData(physx_PxShape const* self_);

    /// Assigns material(s) to the shape. Will remove existing materials from the shape.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
void      physx_PxShape_setMaterials_mut(physx_PxShape* self_, physx_PxMaterial* const* materials, uint16_t materialCount);

    /// Returns the number of materials assigned to the shape.
    ///
    /// You can use [`getMaterials`]() to retrieve the material pointers.
    ///
    /// Number of materials associated with this shape.
uint16_t      physx_PxShape_getNbMaterials(physx_PxShape const* self_);

    /// Retrieve all the material pointers associated with the shape.
    ///
    /// You can retrieve the number of material pointers by calling [`getNbMaterials`]()
    ///
    /// Note: The returned data may contain invalid pointers if you release materials using [`PxMaterial::release`]().
    ///
    /// Number of material pointers written to the buffer.
uint32_t      physx_PxShape_getMaterials(physx_PxShape const* self_, physx_PxMaterial** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Retrieve material from given triangle index.
    ///
    /// The input index is the internal triangle index as used inside the SDK. This is the index
    /// returned to users by various SDK functions such as raycasts.
    ///
    /// This function is only useful for triangle meshes or heightfields, which have per-triangle
    /// materials. For other shapes or SDF triangle meshes, the function returns the single material
    /// associated with the shape, regardless of the index.
    ///
    /// Material from input triangle
    ///
    /// If faceIndex value of 0xFFFFffff is passed as an input for mesh and heightfield shapes, this function will issue a warning and return NULL.
    ///
    /// Scene queries set the value of PxQueryHit::faceIndex to 0xFFFFffff whenever it is undefined or does not apply.
physx_PxBaseMaterial*      physx_PxShape_getMaterialFromInternalFaceIndex(physx_PxShape const* self_, uint32_t faceIndex);

    /// Sets the contact offset.
    ///
    /// Shapes whose distance is less than the sum of their contactOffset values will generate contacts. The contact offset must be positive and
    /// greater than the rest offset. Having a contactOffset greater than than the restOffset allows the collision detection system to
    /// predictively enforce the contact constraint even when the objects are slightly separated. This prevents jitter that would occur
    /// if the constraint were enforced only when shapes were within the rest distance.
    ///
    /// Default:
    /// 0.02f * PxTolerancesScale::length
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
void      physx_PxShape_setContactOffset_mut(physx_PxShape* self_, float contactOffset);

    /// Retrieves the contact offset.
    ///
    /// The contact offset of the shape.
float      physx_PxShape_getContactOffset(physx_PxShape const* self_);

    /// Sets the rest offset.
    ///
    /// Two shapes will come to rest at a distance equal to the sum of their restOffset values. If the restOffset is 0, they should converge to touching
    /// exactly.  Having a restOffset greater than zero is useful to have objects slide smoothly, so that they do not get hung up on irregularities of
    /// each others' surfaces.
    ///
    /// Default:
    /// 0.0f
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
void      physx_PxShape_setRestOffset_mut(physx_PxShape* self_, float restOffset);

    /// Retrieves the rest offset.
    ///
    /// The rest offset of the shape.
float      physx_PxShape_getRestOffset(physx_PxShape const* self_);

    /// Sets the density used to interact with fluids.
    ///
    /// To be physically accurate, the density of a rigid body should be computed as its mass divided by its volume. To
    /// simplify tuning the interaction of fluid and rigid bodies, the density for fluid can differ from the real density. This
    /// allows to create floating bodies, even if they are supposed to sink with their mass and volume.
    ///
    /// Default:
    /// 800.0f
void      physx_PxShape_setDensityForFluid_mut(physx_PxShape* self_, float densityForFluid);

    /// Retrieves the density used to interact with fluids.
    ///
    /// The density of the body when interacting with fluid.
float      physx_PxShape_getDensityForFluid(physx_PxShape const* self_);

    /// Sets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// Default:
    /// 0.0
void      physx_PxShape_setTorsionalPatchRadius_mut(physx_PxShape* self_, float radius);

    /// Gets torsional patch radius.
    ///
    /// This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
    /// will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
    /// so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
    /// rotational friction introduced by the compression of contacting surfaces.
    ///
    /// The torsional patch radius of the shape.
float      physx_PxShape_getTorsionalPatchRadius(physx_PxShape const* self_);

    /// Sets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// Default:
    /// 0.0
void      physx_PxShape_setMinTorsionalPatchRadius_mut(physx_PxShape* self_, float radius);

    /// Gets minimum torsional patch radius.
    ///
    /// This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
    /// that will be applied will be entirely dependent on the value of torsionalPatchRadius.
    ///
    /// If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.
    ///
    /// The minimum torsional patch radius of the shape.
float      physx_PxShape_getMinTorsionalPatchRadius(physx_PxShape const* self_);

    /// Sets shape flags
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the associated actor up automatically.
    ///
    /// Default:
    /// PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE
void      physx_PxShape_setFlag_mut(physx_PxShape* self_, int32_t flag, bool value);

    /// Sets shape flags
void      physx_PxShape_setFlags_mut(physx_PxShape* self_, uint8_t inFlags);

    /// Retrieves shape flags.
    ///
    /// The values of the shape flags.
uint8_t      physx_PxShape_getFlags(physx_PxShape const* self_);

    /// Returns true if the shape is exclusive to an actor.
bool      physx_PxShape_isExclusive(physx_PxShape const* self_);

    /// Sets a name string for the object that can be retrieved with [`getName`]().
    ///
    /// This is for debugging and is not used by the SDK.
    /// The string is not copied by the SDK, only the pointer is stored.
    ///
    /// Default:
    /// NULL
void      physx_PxShape_setName_mut(physx_PxShape* self_, char const* name);

    /// retrieves the name string set with setName().
    ///
    /// The name associated with the shape.
char const*      physx_PxShape_getName(physx_PxShape const* self_);

char const*      physx_PxShape_getConcreteTypeName(physx_PxShape const* self_);

    /// Deletes the rigid actor object.
    ///
    /// Also releases any shapes associated with the actor.
    ///
    /// Releasing an actor will affect any objects that are connected to the actor (constraint shaders like joints etc.).
    /// Such connected objects will be deleted upon scene deletion, or explicitly by the user by calling release()
    /// on these objects. It is recommended to always remove all objects that reference actors before the actors
    /// themselves are removed. It is not possible to retrieve list of dead connected objects.
    ///
    /// Sleeping:
    /// This call will awaken any sleeping actors contacting the deleted actor (directly or indirectly).
    ///
    /// Calls [`PxActor::release`]() so you might want to check the documentation of that method as well.
void      physx_PxRigidActor_release_mut(physx_PxRigidActor* self_);

    /// Returns the internal actor index.
    ///
    /// This is only defined for actors that have been added to a scene.
    ///
    /// The internal actor index, or 0xffffffff if the actor is not part of a scene.
uint32_t      physx_PxRigidActor_getInternalActorIndex(physx_PxRigidActor const* self_);

    /// Retrieves the actors world space transform.
    ///
    /// The getGlobalPose() method retrieves the actor's current actor space to world space transformation.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// Global pose of object.
physx_PxTransform      physx_PxRigidActor_getGlobalPose(physx_PxRigidActor const* self_);

    /// Method for setting an actor's pose in the world.
    ///
    /// This method instantaneously changes the actor space to world space transformation.
    ///
    /// This method is mainly for dynamic rigid bodies (see [`PxRigidDynamic`]). Calling this method on static actors is
    /// likely to result in a performance penalty, since internal optimization structures for static actors may need to be
    /// recomputed. In addition, moving static actors will not interact correctly with dynamic actors or joints.
    ///
    /// To directly control an actor's position and have it correctly interact with dynamic bodies and joints, create a dynamic
    /// body with the PxRigidBodyFlag::eKINEMATIC flag, then use the setKinematicTarget() commands to define its path.
    ///
    /// Even when moving dynamic actors, exercise restraint in making use of this method. Where possible, avoid:
    ///
    /// moving actors into other actors, thus causing overlap (an invalid physical state)
    ///
    /// moving an actor that is connected by a joint to another away from the other (thus causing joint error)
    ///
    /// Sleeping:
    /// This call wakes dynamic actors if they are sleeping and the autowake parameter is true (default).
void      physx_PxRigidActor_setGlobalPose_mut(physx_PxRigidActor* self_, physx_PxTransform const* pose, bool autowake);

    /// Attach a shape to an actor
    ///
    /// This call will increment the reference count of the shape.
    ///
    /// Mass properties of dynamic rigid actors will not automatically be recomputed
    /// to reflect the new mass distribution implied by the shape. Follow this call with a call to
    /// the PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]() to do that.
    ///
    /// Attaching a triangle mesh, heightfield or plane geometry shape configured as eSIMULATION_SHAPE is not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// True if success.
bool      physx_PxRigidActor_attachShape_mut(physx_PxRigidActor* self_, physx_PxShape* shape);

    /// Detach a shape from an actor.
    ///
    /// This will also decrement the reference count of the PxShape, and if the reference count is zero, will cause it to be deleted.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxRigidActor_detachShape_mut(physx_PxRigidActor* self_, physx_PxShape* shape, bool wakeOnLostTouch);

    /// Returns the number of shapes assigned to the actor.
    ///
    /// You can use [`getShapes`]() to retrieve the shape pointers.
    ///
    /// Number of shapes associated with this actor.
uint32_t      physx_PxRigidActor_getNbShapes(physx_PxRigidActor const* self_);

    /// Retrieve all the shape pointers belonging to the actor.
    ///
    /// These are the shapes used by the actor for collision detection.
    ///
    /// You can retrieve the number of shape pointers by calling [`getNbShapes`]()
    ///
    /// Note: Removing shapes with [`PxShape::release`]() will invalidate the pointer of the released shape.
    ///
    /// Number of shape pointers written to the buffer.
uint32_t      physx_PxRigidActor_getShapes(physx_PxRigidActor const* self_, physx_PxShape** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of constraint shaders attached to the actor.
    ///
    /// You can use [`getConstraints`]() to retrieve the constraint shader pointers.
    ///
    /// Number of constraint shaders attached to this actor.
uint32_t      physx_PxRigidActor_getNbConstraints(physx_PxRigidActor const* self_);

    /// Retrieve all the constraint shader pointers belonging to the actor.
    ///
    /// You can retrieve the number of constraint shader pointers by calling [`getNbConstraints`]()
    ///
    /// Note: Removing constraint shaders with [`PxConstraint::release`]() will invalidate the pointer of the released constraint.
    ///
    /// Number of constraint shader pointers written to the buffer.
uint32_t      physx_PxRigidActor_getConstraints(physx_PxRigidActor const* self_, physx_PxConstraint** userBuffer, uint32_t bufferSize, uint32_t startIndex);

physx_PxNodeIndex      physx_PxNodeIndex_new(uint32_t id, uint32_t articLinkId);

physx_PxNodeIndex      physx_PxNodeIndex_new_1(uint32_t id);

uint32_t      physx_PxNodeIndex_index(physx_PxNodeIndex const* self_);

uint32_t      physx_PxNodeIndex_articulationLinkId(physx_PxNodeIndex const* self_);

uint32_t      physx_PxNodeIndex_isArticulation(physx_PxNodeIndex const* self_);

bool      physx_PxNodeIndex_isStaticBody(physx_PxNodeIndex const* self_);

bool      physx_PxNodeIndex_isValid(physx_PxNodeIndex const* self_);

void      physx_PxNodeIndex_setIndices_mut(physx_PxNodeIndex* self_, uint32_t index, uint32_t articLinkId);

void      physx_PxNodeIndex_setIndices_mut_1(physx_PxNodeIndex* self_, uint32_t index);

uint64_t      physx_PxNodeIndex_getInd(physx_PxNodeIndex const* self_);

    /// Sets the pose of the center of mass relative to the actor.
    ///
    /// Changing this transform will not move the actor in the world!
    ///
    /// Setting an unrealistic center of mass which is a long way from the body can make it difficult for
    /// the SDK to solve constraints. Perhaps leading to instability and jittering bodies.
    ///
    /// Default:
    /// the identity transform
void      physx_PxRigidBody_setCMassLocalPose_mut(physx_PxRigidBody* self_, physx_PxTransform const* pose);

    /// Retrieves the center of mass pose relative to the actor frame.
    ///
    /// The center of mass pose relative to the actor frame.
physx_PxTransform      physx_PxRigidBody_getCMassLocalPose(physx_PxRigidBody const* self_);

    /// Sets the mass of a dynamic actor.
    ///
    /// The mass must be non-negative.
    ///
    /// setMass() does not update the inertial properties of the body, to change the inertia tensor
    /// use setMassSpaceInertiaTensor() or the PhysX extensions method [`PxRigidBodyExt::updateMassAndInertia`]().
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic.
    ///
    /// Default:
    /// 1.0
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxRigidBody_setMass_mut(physx_PxRigidBody* self_, float mass);

    /// Retrieves the mass of the actor.
    ///
    /// A value of 0 is interpreted as infinite mass.
    ///
    /// The mass of this actor.
float      physx_PxRigidBody_getMass(physx_PxRigidBody const* self_);

    /// Retrieves the inverse mass of the actor.
    ///
    /// The inverse mass of this actor.
float      physx_PxRigidBody_getInvMass(physx_PxRigidBody const* self_);

    /// Sets the inertia tensor, using a parameter specified in mass space coordinates.
    ///
    /// Note that such matrices are diagonal -- the passed vector is the diagonal.
    ///
    /// If you have a non diagonal world/actor space inertia tensor(3x3 matrix). Then you need to
    /// diagonalize it and set an appropriate mass space transform. See [`setCMassLocalPose`]().
    ///
    /// The inertia tensor elements must be non-negative.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// Values of 0 are not permitted for instances of PxArticulationLink but are permitted for instances of PxRigidDynamic.
    ///
    /// Default:
    /// (1.0, 1.0, 1.0)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxRigidBody_setMassSpaceInertiaTensor_mut(physx_PxRigidBody* self_, physx_PxVec3 const* m);

    /// Retrieves the diagonal inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inertia vector.
    ///
    /// The mass space inertia tensor of this actor.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
physx_PxVec3      physx_PxRigidBody_getMassSpaceInertiaTensor(physx_PxRigidBody const* self_);

    /// Retrieves the diagonal inverse inertia tensor of the actor relative to the mass coordinate frame.
    ///
    /// This method retrieves a mass frame inverse inertia vector.
    ///
    /// A value of 0 in an element is interpreted as infinite inertia along that axis.
    ///
    /// The mass space inverse inertia tensor of this actor.
physx_PxVec3      physx_PxRigidBody_getMassSpaceInvInertiaTensor(physx_PxRigidBody const* self_);

    /// Sets the linear damping coefficient.
    ///
    /// Zero represents no damping. The damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.0
void      physx_PxRigidBody_setLinearDamping_mut(physx_PxRigidBody* self_, float linDamp);

    /// Retrieves the linear damping coefficient.
    ///
    /// The linear damping coefficient associated with this actor.
float      physx_PxRigidBody_getLinearDamping(physx_PxRigidBody const* self_);

    /// Sets the angular damping coefficient.
    ///
    /// Zero represents no damping.
    ///
    /// The angular damping coefficient must be nonnegative.
    ///
    /// Default:
    /// 0.05
void      physx_PxRigidBody_setAngularDamping_mut(physx_PxRigidBody* self_, float angDamp);

    /// Retrieves the angular damping coefficient.
    ///
    /// The angular damping coefficient associated with this actor.
float      physx_PxRigidBody_getAngularDamping(physx_PxRigidBody const* self_);

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
physx_PxVec3      physx_PxRigidBody_getLinearVelocity(physx_PxRigidBody const* self_);

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
physx_PxVec3      physx_PxRigidBody_getAngularVelocity(physx_PxRigidBody const* self_);

    /// Lets you set the maximum linear velocity permitted for this actor.
    ///
    /// With this function, you can set the  maximum linear velocity permitted for this rigid body.
    /// Higher angular velocities are clamped to this value.
    ///
    /// Note: The angular velocity is clamped to the set value
    /// before
    /// the solver, which means that
    /// the limit may still be momentarily exceeded.
    ///
    /// Default:
    /// PX_MAX_F32
void      physx_PxRigidBody_setMaxLinearVelocity_mut(physx_PxRigidBody* self_, float maxLinVel);

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
float      physx_PxRigidBody_getMaxLinearVelocity(physx_PxRigidBody const* self_);

    /// Lets you set the maximum angular velocity permitted for this actor.
    ///
    /// For various internal computations, very quickly rotating actors introduce error
    /// into the simulation, which leads to undesired results.
    ///
    /// With this function, you can set the  maximum angular velocity permitted for this rigid body.
    /// Higher angular velocities are clamped to this value.
    ///
    /// Note: The angular velocity is clamped to the set value
    /// before
    /// the solver, which means that
    /// the limit may still be momentarily exceeded.
    ///
    /// Default:
    /// 100.0
void      physx_PxRigidBody_setMaxAngularVelocity_mut(physx_PxRigidBody* self_, float maxAngVel);

    /// Retrieves the maximum angular velocity permitted for this actor.
    ///
    /// The maximum allowed angular velocity for this actor.
float      physx_PxRigidBody_getMaxAngularVelocity(physx_PxRigidBody const* self_);

    /// Applies a force (or impulse) defined in the global coordinate frame to the actor at its center of mass.
    ///
    /// This will not induce a torque
    /// .
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive.
    ///
    /// Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators and are just short hand for multiplying the vector parameter by inverse mass and then using PxForceMode::eACCELERATION and
    /// PxForceMode::eVELOCITY_CHANGE respectively.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// if this is called on an articulation link, only the link is updated, not the entire articulation.
    ///
    /// see [`PxRigidBodyExt::computeVelocityDeltaFromImpulse`] for details of how to compute the change in linear velocity that
    /// will arise from the application of an impulsive force, where an impulsive force is applied force multiplied by a timestep.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the force is non-zero.
void      physx_PxRigidBody_addForce_mut(physx_PxRigidBody* self_, physx_PxVec3 const* force, int32_t mode, bool autowake);

    /// Applies an impulsive torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the torque is to be conventional or impulsive.
    ///
    /// Each actor has an angular acceleration and an angular velocity change accumulator which are directly modified using the modes
    /// PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE
    /// also modify these same accumulators and are just short hand for multiplying the vector parameter by inverse inertia and then
    /// using PxForceMode::eACCELERATION and PxForceMode::eVELOCITY_CHANGE respectively.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// if this called on an articulation link, only the link is updated, not the entire articulation.
    ///
    /// see [`PxRigidBodyExt::computeVelocityDeltaFromImpulse`] for details of how to compute the change in angular velocity that
    /// will arise from the application of an impulsive torque, where an impulsive torque is an applied torque multiplied by a timestep.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the torque is non-zero.
void      physx_PxRigidBody_addTorque_mut(physx_PxRigidBody* self_, physx_PxVec3 const* torque, int32_t mode, bool autowake);

    /// Clears the accumulated forces (sets the accumulated force back to zero).
    ///
    /// Each actor has an acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators (see PxRigidBody::addForce() for details); therefore the effect of calling clearForce(PxForceMode::eFORCE) is equivalent to calling
    /// clearForce(PxForceMode::eACCELERATION), and the effect of calling clearForce(PxForceMode::eIMPULSE) is equivalent to calling
    /// clearForce(PxForceMode::eVELOCITY_CHANGE).
    ///
    /// ::PxForceMode determines if the cleared force is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
void      physx_PxRigidBody_clearForce_mut(physx_PxRigidBody* self_, int32_t mode);

    /// Clears the impulsive torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// Each actor has an angular acceleration and a velocity change accumulator which are directly modified using the modes PxForceMode::eACCELERATION
    /// and PxForceMode::eVELOCITY_CHANGE respectively.  The modes PxForceMode::eFORCE and PxForceMode::eIMPULSE also modify these same
    /// accumulators (see PxRigidBody::addTorque() for details); therefore the effect of calling clearTorque(PxForceMode::eFORCE) is equivalent to calling
    /// clearTorque(PxForceMode::eACCELERATION), and the effect of calling clearTorque(PxForceMode::eIMPULSE) is equivalent to calling
    /// clearTorque(PxForceMode::eVELOCITY_CHANGE).
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
void      physx_PxRigidBody_clearTorque_mut(physx_PxRigidBody* self_, int32_t mode);

    /// Sets the impulsive force and torque defined in the global coordinate frame to the actor.
    ///
    /// ::PxForceMode determines if the cleared torque is to be conventional or impulsive.
    ///
    /// The force modes PxForceMode::eIMPULSE and PxForceMode::eVELOCITY_CHANGE can not be applied to articulation links.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
void      physx_PxRigidBody_setForceAndTorque_mut(physx_PxRigidBody* self_, physx_PxVec3 const* force, physx_PxVec3 const* torque, int32_t mode);

    /// Raises or clears a particular rigid body flag.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// Default:
    /// no flags are set
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxRigidBody_setRigidBodyFlag_mut(physx_PxRigidBody* self_, int32_t flag, bool value);

void      physx_PxRigidBody_setRigidBodyFlags_mut(physx_PxRigidBody* self_, uint16_t inFlags);

    /// Reads the PxRigidBody flags.
    ///
    /// See the list of flags [`PxRigidBodyFlag`]
    ///
    /// The values of the PxRigidBody flags.
uint16_t      physx_PxRigidBody_getRigidBodyFlags(physx_PxRigidBody const* self_);

    /// Sets the CCD minimum advance coefficient.
    ///
    /// The CCD minimum advance coefficient is a value in the range [0, 1] that is used to control the minimum amount of time a body is integrated when
    /// it has a CCD contact. The actual minimum amount of time that is integrated depends on various properties, including the relative speed and collision shapes
    /// of the bodies involved in the contact. From these properties, a numeric value is calculated that determines the maximum distance (and therefore maximum time)
    /// which these bodies could be integrated forwards that would ensure that these bodies did not pass through each-other. This value is then scaled by CCD minimum advance
    /// coefficient to determine the amount of time that will be consumed in the CCD pass.
    ///
    /// Things to consider:
    /// A large value (approaching 1) ensures that the objects will always advance some time. However, larger values increase the chances of objects gently drifting through each-other in
    /// scenes which the constraint solver can't converge, e.g. scenes where an object is being dragged through a wall with a constraint.
    /// A value of 0 ensures that the pair of objects stop at the exact time-of-impact and will not gently drift through each-other. However, with very small/thin objects initially in
    /// contact, this can lead to a large amount of time being dropped and increases the chances of jamming. Jamming occurs when the an object is persistently in contact with an object
    /// such that the time-of-impact is 0, which results in no time being advanced for those objects in that CCD pass.
    ///
    /// The chances of jamming can be reduced by increasing the number of CCD mass
void      physx_PxRigidBody_setMinCCDAdvanceCoefficient_mut(physx_PxRigidBody* self_, float advanceCoefficient);

    /// Gets the CCD minimum advance coefficient.
    ///
    /// The value of the CCD min advance coefficient.
float      physx_PxRigidBody_getMinCCDAdvanceCoefficient(physx_PxRigidBody const* self_);

    /// Sets the maximum depenetration velocity permitted to be introduced by the solver.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
void      physx_PxRigidBody_setMaxDepenetrationVelocity_mut(physx_PxRigidBody* self_, float biasClamp);

    /// Returns the maximum depenetration velocity the solver is permitted to introduced.
    /// This value controls how much velocity the solver can introduce to correct for penetrations in contacts.
    ///
    /// The maximum penetration bias applied by the solver.
float      physx_PxRigidBody_getMaxDepenetrationVelocity(physx_PxRigidBody const* self_);

    /// Sets a limit on the impulse that may be applied at a contact. The maximum impulse at a contact between two dynamic or kinematic
    /// bodies will be the minimum of the two limit values. For a collision between a static and a dynamic body, the impulse is limited
    /// by the value for the dynamic body.
void      physx_PxRigidBody_setMaxContactImpulse_mut(physx_PxRigidBody* self_, float maxImpulse);

    /// Returns the maximum impulse that may be applied at a contact.
    ///
    /// The maximum impulse that may be applied at a contact
float      physx_PxRigidBody_getMaxContactImpulse(physx_PxRigidBody const* self_);

    /// Sets a distance scale whereby the angular influence of a contact on the normal constraint in a contact is
    /// zeroed if normal.cross(offset) falls below this tolerance. Rather than acting as an absolute value, this tolerance
    /// is scaled by the ratio rXn.dot(angVel)/normal.dot(linVel) such that contacts that have relatively larger angular velocity
    /// than linear normal velocity (e.g. rolling wheels) achieve larger slop values as the angular velocity increases.
void      physx_PxRigidBody_setContactSlopCoefficient_mut(physx_PxRigidBody* self_, float slopCoefficient);

    /// Returns the contact slop coefficient.
    ///
    /// The contact slop coefficient.
float      physx_PxRigidBody_getContactSlopCoefficient(physx_PxRigidBody const* self_);

    /// Returns the island node index
    ///
    /// The island node index.
physx_PxNodeIndex      physx_PxRigidBody_getInternalIslandNodeIndex(physx_PxRigidBody const* self_);

    /// Releases the link from the articulation.
    ///
    /// Only a leaf articulation link can be released.
    ///
    /// Releasing a link is not allowed while the articulation link is in a scene. In order to release a link,
    /// remove and then re-add the corresponding articulation to the scene.
void      physx_PxArticulationLink_release_mut(physx_PxArticulationLink* self_);

    /// Gets the articulation that the link is a part of.
    ///
    /// The articulation.
physx_PxArticulationReducedCoordinate*      physx_PxArticulationLink_getArticulation(physx_PxArticulationLink const* self_);

    /// Gets the joint which connects this link to its parent.
    ///
    /// The joint connecting the link to the parent. NULL for the root link.
physx_PxArticulationJointReducedCoordinate*      physx_PxArticulationLink_getInboundJoint(physx_PxArticulationLink const* self_);

    /// Gets the number of degrees of freedom of the joint which connects this link to its parent.
    ///
    /// - The root link DOF-count is defined to be 0 regardless of PxArticulationFlag::eFIX_BASE.
    /// - The return value is only valid for articulations that are in a scene.
    ///
    /// The number of degrees of freedom, or 0xFFFFFFFF if the articulation is not in a scene.
uint32_t      physx_PxArticulationLink_getInboundJointDof(physx_PxArticulationLink const* self_);

    /// Gets the number of child links.
    ///
    /// The number of child links.
uint32_t      physx_PxArticulationLink_getNbChildren(physx_PxArticulationLink const* self_);

    /// Gets the low-level link index that may be used to index into members of PxArticulationCache.
    ///
    /// The return value is only valid for articulations that are in a scene.
    ///
    /// The low-level index, or 0xFFFFFFFF if the articulation is not in a scene.
uint32_t      physx_PxArticulationLink_getLinkIndex(physx_PxArticulationLink const* self_);

    /// Retrieves the child links.
    ///
    /// The number of articulation links written to the buffer.
uint32_t      physx_PxArticulationLink_getChildren(physx_PxArticulationLink const* self_, physx_PxArticulationLink** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Set the constraint-force-mixing scale term.
    ///
    /// The cfm scale term is a stabilization term that helps avoid instabilities with over-constrained
    /// configurations. It should be a small value that is multiplied by 1/mass internally to produce
    /// an additional bias added to the unit response term in the solver.
    ///
    /// Default:
    /// 0.025
    /// Range:
    /// [0, 1]
    ///
    /// This call is not allowed while the simulation is running.
void      physx_PxArticulationLink_setCfmScale_mut(physx_PxArticulationLink* self_, float cfm);

    /// Get the constraint-force-mixing scale term.
    ///
    /// The constraint-force-mixing scale term.
float      physx_PxArticulationLink_getCfmScale(physx_PxArticulationLink const* self_);

    /// Get the linear velocity of the link.
    ///
    /// - The linear velocity is with respect to the link's center of mass and not the actor frame origin.
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The linear velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
physx_PxVec3      physx_PxArticulationLink_getLinearVelocity(physx_PxArticulationLink const* self_);

    /// Get the angular velocity of the link.
    ///
    /// - For performance, prefer PxArticulationCache::linkVelocity to get link spatial velocities in a batch query.
    /// - When the articulation state is updated via non-cache API, use PxArticulationReducedCoordinate::updateKinematic before querying velocity.
    ///
    /// The angular velocity of the link.
    ///
    /// This call is not allowed while the simulation is running except in a split simulation during [`PxScene::collide`]() and up to #PxScene::advance(),
    /// and in PxContactModifyCallback or in contact report callbacks.
physx_PxVec3      physx_PxArticulationLink_getAngularVelocity(physx_PxArticulationLink const* self_);

    /// Returns the string name of the dynamic type.
    ///
    /// The string name.
char const*      physx_PxArticulationLink_getConcreteTypeName(physx_PxArticulationLink const* self_);

physx_PxConeLimitedConstraint      physx_PxConeLimitedConstraint_new();

    /// Releases a PxConstraint instance.
    ///
    /// This call does not wake up the connected rigid bodies.
void      physx_PxConstraint_release_mut(physx_PxConstraint* self_);

    /// Retrieves the scene which this constraint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
physx_PxScene*      physx_PxConstraint_getScene(physx_PxConstraint const* self_);

    /// Retrieves the actors for this constraint.
void      physx_PxConstraint_getActors(physx_PxConstraint const* self_, physx_PxRigidActor** actor0, physx_PxRigidActor** actor1);

    /// Sets the actors for this constraint.
void      physx_PxConstraint_setActors_mut(physx_PxConstraint* self_, physx_PxRigidActor* actor0, physx_PxRigidActor* actor1);

    /// Notify the scene that the constraint shader data has been updated by the application
void      physx_PxConstraint_markDirty_mut(physx_PxConstraint* self_);

    /// Retrieve the flags for this constraint
    ///
    /// the constraint flags
uint16_t      physx_PxConstraint_getFlags(physx_PxConstraint const* self_);

    /// Set the flags for this constraint
    ///
    /// default: PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
void      physx_PxConstraint_setFlags_mut(physx_PxConstraint* self_, uint16_t flags);

    /// Set a flag for this constraint
void      physx_PxConstraint_setFlag_mut(physx_PxConstraint* self_, int32_t flag, bool value);

    /// Retrieve the constraint force most recently applied to maintain this constraint.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
void      physx_PxConstraint_getForce(physx_PxConstraint const* self_, physx_PxVec3* linear, physx_PxVec3* angular);

    /// whether the constraint is valid.
    ///
    /// A constraint is valid if it has at least one dynamic rigid body or articulation link. A constraint that
    /// is not valid may not be inserted into a scene, and therefore a static actor to which an invalid constraint
    /// is attached may not be inserted into a scene.
    ///
    /// Invalid constraints arise only when an actor to which the constraint is attached has been deleted.
bool      physx_PxConstraint_isValid(physx_PxConstraint const* self_);

    /// Set the break force and torque thresholds for this constraint.
    ///
    /// If either the force or torque measured at the constraint exceed these thresholds the constraint will break.
void      physx_PxConstraint_setBreakForce_mut(physx_PxConstraint* self_, float linear, float angular);

    /// Retrieve the constraint break force and torque thresholds
void      physx_PxConstraint_getBreakForce(physx_PxConstraint const* self_, float* linear, float* angular);

    /// Set the minimum response threshold for a constraint row
    ///
    /// When using mass modification for a joint or infinite inertia for a jointed body, very stiff solver constraints can be generated which
    /// can destabilize simulation. Setting this value to a small positive value (e.g. 1e-8) will cause constraint rows to be ignored if very
    /// large changes in impulses will generate only small changes in velocity. When setting this value, also set
    /// PxConstraintFlag::eDISABLE_PREPROCESSING. The solver accuracy for this joint may be reduced.
void      physx_PxConstraint_setMinResponseThreshold_mut(physx_PxConstraint* self_, float threshold);

    /// Retrieve the constraint break force and torque thresholds
    ///
    /// the minimum response threshold for a constraint row
float      physx_PxConstraint_getMinResponseThreshold(physx_PxConstraint const* self_);

    /// Fetch external owner of the constraint.
    ///
    /// Provides a reference to the external owner of a constraint and a unique owner type ID.
    ///
    /// Reference to the external object which owns the constraint.
void*      physx_PxConstraint_getExternalReference_mut(physx_PxConstraint* self_, uint32_t* typeID);

    /// Set the constraint functions for this constraint
void      physx_PxConstraint_setConstraintFunctions_mut(physx_PxConstraint* self_, physx_PxConstraintConnector* connector, physx_PxConstraintShaderTable const* shaders);

char const*      physx_PxConstraint_getConcreteTypeName(physx_PxConstraint const* self_);

    /// Constructor
physx_PxContactStreamIterator      physx_PxContactStreamIterator_new(uint8_t const* contactPatches, uint8_t const* contactPoints, uint32_t const* contactFaceIndices, uint32_t nbPatches, uint32_t nbContacts);

    /// Returns whether there are more patches in this stream.
    ///
    /// Whether there are more patches in this stream.
bool      physx_PxContactStreamIterator_hasNextPatch(physx_PxContactStreamIterator const* self_);

    /// Returns the total contact count.
    ///
    /// Total contact count.
uint32_t      physx_PxContactStreamIterator_getTotalContactCount(physx_PxContactStreamIterator const* self_);

    /// Returns the total patch count.
    ///
    /// Total patch count.
uint32_t      physx_PxContactStreamIterator_getTotalPatchCount(physx_PxContactStreamIterator const* self_);

    /// Advances iterator to next contact patch.
void      physx_PxContactStreamIterator_nextPatch_mut(physx_PxContactStreamIterator* self_);

    /// Returns if the current patch has more contacts.
    ///
    /// If there are more contacts in the current patch.
bool      physx_PxContactStreamIterator_hasNextContact(physx_PxContactStreamIterator const* self_);

    /// Advances to the next contact in the patch.
void      physx_PxContactStreamIterator_nextContact_mut(physx_PxContactStreamIterator* self_);

    /// Gets the current contact's normal
    ///
    /// The current contact's normal.
physx_PxVec3 const*      physx_PxContactStreamIterator_getContactNormal(physx_PxContactStreamIterator const* self_);

    /// Gets the inverse mass scale for body 0.
    ///
    /// The inverse mass scale for body 0.
float      physx_PxContactStreamIterator_getInvMassScale0(physx_PxContactStreamIterator const* self_);

    /// Gets the inverse mass scale for body 1.
    ///
    /// The inverse mass scale for body 1.
float      physx_PxContactStreamIterator_getInvMassScale1(physx_PxContactStreamIterator const* self_);

    /// Gets the inverse inertia scale for body 0.
    ///
    /// The inverse inertia scale for body 0.
float      physx_PxContactStreamIterator_getInvInertiaScale0(physx_PxContactStreamIterator const* self_);

    /// Gets the inverse inertia scale for body 1.
    ///
    /// The inverse inertia scale for body 1.
float      physx_PxContactStreamIterator_getInvInertiaScale1(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's max impulse.
    ///
    /// The contact's max impulse.
float      physx_PxContactStreamIterator_getMaxImpulse(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's target velocity.
    ///
    /// The contact's target velocity.
physx_PxVec3 const*      physx_PxContactStreamIterator_getTargetVel(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's contact point.
    ///
    /// The contact's contact point.
physx_PxVec3 const*      physx_PxContactStreamIterator_getContactPoint(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's separation.
    ///
    /// The contact's separation.
float      physx_PxContactStreamIterator_getSeparation(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's face index for shape 0.
    ///
    /// The contact's face index for shape 0.
uint32_t      physx_PxContactStreamIterator_getFaceIndex0(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's face index for shape 1.
    ///
    /// The contact's face index for shape 1.
uint32_t      physx_PxContactStreamIterator_getFaceIndex1(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's static friction coefficient.
    ///
    /// The contact's static friction coefficient.
float      physx_PxContactStreamIterator_getStaticFriction(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's dynamic friction coefficient.
    ///
    /// The contact's dynamic friction coefficient.
float      physx_PxContactStreamIterator_getDynamicFriction(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's restitution coefficient.
    ///
    /// The contact's restitution coefficient.
float      physx_PxContactStreamIterator_getRestitution(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's damping value.
    ///
    /// The contact's damping value.
float      physx_PxContactStreamIterator_getDamping(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's material flags.
    ///
    /// The contact's material flags.
uint32_t      physx_PxContactStreamIterator_getMaterialFlags(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's material index for shape 0.
    ///
    /// The contact's material index for shape 0.
uint16_t      physx_PxContactStreamIterator_getMaterialIndex0(physx_PxContactStreamIterator const* self_);

    /// Gets the contact's material index for shape 1.
    ///
    /// The contact's material index for shape 1.
uint16_t      physx_PxContactStreamIterator_getMaterialIndex1(physx_PxContactStreamIterator const* self_);

    /// Advances the contact stream iterator to a specific contact index.
    ///
    /// True if advancing was possible
bool      physx_PxContactStreamIterator_advanceToIndex_mut(physx_PxContactStreamIterator* self_, uint32_t initialIndex);

    /// Get the position of a specific contact point in the set.
    ///
    /// Position to the requested point in world space
physx_PxVec3 const*      physx_PxContactSet_getPoint(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the position of a specific contact point in the set.
void      physx_PxContactSet_setPoint_mut(physx_PxContactSet* self_, uint32_t i, physx_PxVec3 const* p);

    /// Get the contact normal of a specific contact point in the set.
    ///
    /// The requested normal in world space
physx_PxVec3 const*      physx_PxContactSet_getNormal(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the contact normal of a specific contact point in the set.
    ///
    /// Changing the normal can cause contact points to be ignored.
void      physx_PxContactSet_setNormal_mut(physx_PxContactSet* self_, uint32_t i, physx_PxVec3 const* n);

    /// Get the separation distance of a specific contact point in the set.
    ///
    /// The separation. Negative implies penetration.
float      physx_PxContactSet_getSeparation(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the separation of a specific contact point in the set.
void      physx_PxContactSet_setSeparation_mut(physx_PxContactSet* self_, uint32_t i, float s);

    /// Get the target velocity of a specific contact point in the set.
    ///
    /// The target velocity in world frame
physx_PxVec3 const*      physx_PxContactSet_getTargetVelocity(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the target velocity of a specific contact point in the set.
void      physx_PxContactSet_setTargetVelocity_mut(physx_PxContactSet* self_, uint32_t i, physx_PxVec3 const* v);

    /// Get the face index with respect to the first shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the first shape
    ///
    /// At the moment, the first shape is never a tri-mesh, therefore this function always returns PXC_CONTACT_NO_FACE_INDEX
uint32_t      physx_PxContactSet_getInternalFaceIndex0(physx_PxContactSet const* self_, uint32_t i);

    /// Get the face index with respect to the second shape of the pair for a specific contact point in the set.
    ///
    /// The face index of the second shape
uint32_t      physx_PxContactSet_getInternalFaceIndex1(physx_PxContactSet const* self_, uint32_t i);

    /// Get the maximum impulse for a specific contact point in the set.
    ///
    /// The maximum impulse
float      physx_PxContactSet_getMaxImpulse(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the maximum impulse for a specific contact point in the set.
    ///
    /// Must be nonnegative. If set to zero, the contact point will be ignored
void      physx_PxContactSet_setMaxImpulse_mut(physx_PxContactSet* self_, uint32_t i, float s);

    /// Get the restitution coefficient for a specific contact point in the set.
    ///
    /// The restitution coefficient
float      physx_PxContactSet_getRestitution(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the restitution coefficient for a specific contact point in the set.
    ///
    /// Valid ranges [0,1]
void      physx_PxContactSet_setRestitution_mut(physx_PxContactSet* self_, uint32_t i, float r);

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient (dimensionless)
float      physx_PxContactSet_getStaticFriction(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the static friction coefficient for a specific contact point in the set.
void      physx_PxContactSet_setStaticFriction_mut(physx_PxContactSet* self_, uint32_t i, float f);

    /// Get the static friction coefficient for a specific contact point in the set.
    ///
    /// The friction coefficient
float      physx_PxContactSet_getDynamicFriction(physx_PxContactSet const* self_, uint32_t i);

    /// Alter the static dynamic coefficient for a specific contact point in the set.
void      physx_PxContactSet_setDynamicFriction_mut(physx_PxContactSet* self_, uint32_t i, float f);

    /// Ignore the contact point.
    ///
    /// If a contact point is ignored then no force will get applied at this point. This can be used to disable collision in certain areas of a shape, for example.
void      physx_PxContactSet_ignore_mut(physx_PxContactSet* self_, uint32_t i);

    /// The number of contact points in the set.
uint32_t      physx_PxContactSet_size(physx_PxContactSet const* self_);

    /// Returns the invMassScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
float      physx_PxContactSet_getInvMassScale0(physx_PxContactSet const* self_);

    /// Returns the invMassScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
float      physx_PxContactSet_getInvMassScale1(physx_PxContactSet const* self_);

    /// Returns the invInertiaScale of body 0
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
float      physx_PxContactSet_getInvInertiaScale0(physx_PxContactSet const* self_);

    /// Returns the invInertiaScale of body 1
    ///
    /// A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
float      physx_PxContactSet_getInvInertiaScale1(physx_PxContactSet const* self_);

    /// Sets the invMassScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
void      physx_PxContactSet_setInvMassScale0_mut(physx_PxContactSet* self_, float scale);

    /// Sets the invMassScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger mass. A value of 0.f makes this contact
    /// treat the body as if it had infinite mass. Any value > 1.f makes this contact treat the body as if it had smaller mass.
void      physx_PxContactSet_setInvMassScale1_mut(physx_PxContactSet* self_, float scale);

    /// Sets the invInertiaScale of body 0
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
void      physx_PxContactSet_setInvInertiaScale0_mut(physx_PxContactSet* self_, float scale);

    /// Sets the invInertiaScale of body 1
    ///
    /// This can be set to any value in the range [0, PX_MAX_F32). A value
    /// <
    /// 1.0 makes this contact treat the body as if it had larger inertia. A value of 0.f makes this contact
    /// treat the body as if it had infinite inertia. Any value > 1.f makes this contact treat the body as if it had smaller inertia.
void      physx_PxContactSet_setInvInertiaScale1_mut(physx_PxContactSet* self_, float scale);

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
void      physx_PxContactModifyCallback_onContactModify_mut(physx_PxContactModifyCallback* self_, physx_PxContactModifyPair*const pairs, uint32_t count);

    /// Passes modifiable arrays of contacts to the application.
    ///
    /// The initial contacts are regenerated from scratch each frame by collision detection.
    ///
    /// The number of contacts can not be changed, so you cannot add your own contacts.  You may however
    /// disable contacts using PxContactSet::ignore().
void      physx_PxCCDContactModifyCallback_onCCDContactModify_mut(physx_PxCCDContactModifyCallback* self_, physx_PxContactModifyPair*const pairs, uint32_t count);

    /// Notification if an object or its memory gets released
    ///
    /// If release() gets called on a PxBase object, an eUSER_RELEASE event will get fired immediately. The object state can be queried in the callback but
    /// it is not allowed to change the state. Furthermore, when reading from the object it is the user's responsibility to make sure that no other thread
    /// is writing at the same time to the object (this includes the simulation itself, i.e., [`PxScene::fetchResults`]() must not get called at the same time).
    ///
    /// Calling release() on a PxBase object does not necessarily trigger its destructor immediately. For example, the object can be shared and might still
    /// be referenced by other objects or the simulation might still be running and accessing the object state. In such cases the destructor will be called
    /// as soon as it is safe to do so. After the destruction of the object and its memory, an eMEMORY_RELEASE event will get fired. In this case it is not
    /// allowed to dereference the object pointer in the callback.
void      physx_PxDeletionListener_onRelease_mut(physx_PxDeletionListener* self_, physx_PxBase const* observed, void* userData, int32_t deletionEvent);

bool      physx_PxBaseMaterial_isKindOf(physx_PxBaseMaterial const* self_, char const* name);

    /// Sets young's modulus which defines the body's stiffness
void      physx_PxFEMMaterial_setYoungsModulus_mut(physx_PxFEMMaterial* self_, float young);

    /// Retrieves the young's modulus value.
    ///
    /// The young's modulus value.
float      physx_PxFEMMaterial_getYoungsModulus(physx_PxFEMMaterial const* self_);

    /// Sets the Poisson's ratio which defines the body's volume preservation. Completely incompressible materials have a poisson ratio of 0.5. Its value should not be set to exactly 0.5 because this leads to numerical problems.
void      physx_PxFEMMaterial_setPoissons_mut(physx_PxFEMMaterial* self_, float poisson);

    /// Retrieves the Poisson's ratio.
    ///
    /// The Poisson's ratio.
float      physx_PxFEMMaterial_getPoissons(physx_PxFEMMaterial const* self_);

    /// Sets the dynamic friction value which defines the strength of resistance when two objects slide relative to each other while in contact.
void      physx_PxFEMMaterial_setDynamicFriction_mut(physx_PxFEMMaterial* self_, float dynamicFriction);

    /// Retrieves the dynamic friction value
    ///
    /// The dynamic friction value
float      physx_PxFEMMaterial_getDynamicFriction(physx_PxFEMMaterial const* self_);

physx_PxFilterData      physx_PxFilterData_new(int32_t anon_param0);

    /// Default constructor.
physx_PxFilterData      physx_PxFilterData_new_1();

    /// Constructor to set filter data initially.
physx_PxFilterData      physx_PxFilterData_new_2(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3);

    /// (re)sets the structure to the default.
void      physx_PxFilterData_setToDefault_mut(physx_PxFilterData* self_);

    /// Extract filter object type from the filter attributes of a collision pair object
    ///
    /// The type of the collision pair object.
int32_t      physx_phys_PxGetFilterObjectType(uint32_t attr);

    /// Specifies whether the collision object belongs to a kinematic rigid body
    ///
    /// True if the object belongs to a kinematic rigid body, else false
bool      physx_phys_PxFilterObjectIsKinematic(uint32_t attr);

    /// Specifies whether the collision object is a trigger shape
    ///
    /// True if the object is a trigger shape, else false
bool      physx_phys_PxFilterObjectIsTrigger(uint32_t attr);

    /// Filter method to specify how a pair of potentially colliding objects should be processed.
    ///
    /// This method gets called when the filter flags returned by the filter shader (see [`PxSimulationFilterShader`])
    /// indicate that the filter callback should be invoked ([`PxFilterFlag::eCALLBACK`] or #PxFilterFlag::eNOTIFY set).
    /// Return the PxFilterFlag flags and set the PxPairFlag flags to define what the simulation should do with the given
    /// collision pair.
    ///
    /// Filter flags defining whether the pair should be discarded, temporarily ignored or processed and whether the pair
    /// should be tracked and send a report on pair deletion through the filter callback
uint16_t      physx_PxSimulationFilterCallback_pairFound_mut(physx_PxSimulationFilterCallback* self_, uint32_t pairID, uint32_t attributes0, physx_PxFilterData filterData0, physx_PxActor const* a0, physx_PxShape const* s0, uint32_t attributes1, physx_PxFilterData filterData1, physx_PxActor const* a1, physx_PxShape const* s1, uint16_t* pairFlags);

    /// Callback to inform that a tracked collision pair is gone.
    ///
    /// This method gets called when a collision pair disappears or gets re-filtered. Only applies to
    /// collision pairs which have been marked as filter callback pairs ([`PxFilterFlag::eNOTIFY`] set in #pairFound()).
void      physx_PxSimulationFilterCallback_pairLost_mut(physx_PxSimulationFilterCallback* self_, uint32_t pairID, uint32_t attributes0, physx_PxFilterData filterData0, uint32_t attributes1, physx_PxFilterData filterData1, bool objectRemoved);

    /// Callback to give the opportunity to change the filter state of a tracked collision pair.
    ///
    /// This method gets called once per simulation step to let the application change the filter and pair
    /// flags of a collision pair that has been reported in [`pairFound`]() and requested callbacks by
    /// setting [`PxFilterFlag::eNOTIFY`]. To request a change of filter status, the target pair has to be
    /// specified by its ID, the new filter and pair flags have to be provided and the method should return true.
    ///
    /// If this method changes the filter status of a collision pair and the pair should keep being tracked
    /// by the filter callbacks then [`PxFilterFlag::eNOTIFY`] has to be set.
    ///
    /// The application is responsible to ensure that this method does not get called for pairs that have been
    /// reported as lost, see [`pairLost`]().
    ///
    /// True if the changes should be applied. In this case the method will get called again. False if
    /// no more status changes should be done in the current simulation step. In that case the provided flags will be discarded.
bool      physx_PxSimulationFilterCallback_statusChange_mut(physx_PxSimulationFilterCallback* self_, uint32_t* pairID, uint16_t* pairFlags, uint16_t* filterFlags);

    /// Any combination of PxDataAccessFlag::eREADABLE and PxDataAccessFlag::eWRITABLE
uint8_t      physx_PxLockedData_getDataAccessFlags_mut(physx_PxLockedData* self_);

    /// Unlocks the bulk data.
void      physx_PxLockedData_unlock_mut(physx_PxLockedData* self_);

    /// virtual destructor
void      physx_PxLockedData_delete(physx_PxLockedData* self_);

    /// Sets the coefficient of dynamic friction.
    ///
    /// The coefficient of dynamic friction should be in [0, PX_MAX_F32). If set to greater than staticFriction, the effective value of staticFriction will be increased to match.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setDynamicFriction_mut(physx_PxMaterial* self_, float coef);

    /// Retrieves the DynamicFriction value.
    ///
    /// The coefficient of dynamic friction.
float      physx_PxMaterial_getDynamicFriction(physx_PxMaterial const* self_);

    /// Sets the coefficient of static friction
    ///
    /// The coefficient of static friction should be in the range [0, PX_MAX_F32)
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setStaticFriction_mut(physx_PxMaterial* self_, float coef);

    /// Retrieves the coefficient of static friction.
    ///
    /// The coefficient of static friction.
float      physx_PxMaterial_getStaticFriction(physx_PxMaterial const* self_);

    /// Sets the coefficient of restitution
    ///
    /// A coefficient of 0 makes the object bounce as little as possible, higher values up to 1.0 result in more bounce.
    ///
    /// This property is overloaded when PxMaterialFlag::eCOMPLIANT_CONTACT flag is enabled. This permits negative values for restitution to be provided.
    /// The negative values are converted into spring stiffness terms for an implicit spring simulated at the contact site, with the spring positional error defined by
    /// the contact separation value. Higher stiffness terms produce stiffer springs that behave more like a rigid contact.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setRestitution_mut(physx_PxMaterial* self_, float rest);

    /// Retrieves the coefficient of restitution.
    ///
    /// See [`setRestitution`].
    ///
    /// The coefficient of restitution.
float      physx_PxMaterial_getRestitution(physx_PxMaterial const* self_);

    /// Sets the coefficient of damping
    ///
    /// This property only affects the simulation if PxMaterialFlag::eCOMPLIANT_CONTACT is raised.
    /// Damping works together with spring stiffness (set through a negative restitution value). Spring stiffness corrects positional error while
    /// damping resists relative velocity. Setting a high damping coefficient can produce spongy contacts.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setDamping_mut(physx_PxMaterial* self_, float damping);

    /// Retrieves the coefficient of damping.
    ///
    /// See [`setDamping`].
    ///
    /// The coefficient of damping.
float      physx_PxMaterial_getDamping(physx_PxMaterial const* self_);

    /// Raises or clears a particular material flag.
    ///
    /// See the list of flags [`PxMaterialFlag`]
    ///
    /// Default:
    /// eIMPROVED_PATCH_FRICTION
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setFlag_mut(physx_PxMaterial* self_, int32_t flag, bool b);

    /// sets all the material flags.
    ///
    /// See the list of flags [`PxMaterialFlag`]
    ///
    /// Default:
    /// eIMPROVED_PATCH_FRICTION
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setFlags_mut(physx_PxMaterial* self_, uint16_t flags);

    /// Retrieves the flags. See [`PxMaterialFlag`].
    ///
    /// The material flags.
uint16_t      physx_PxMaterial_getFlags(physx_PxMaterial const* self_);

    /// Sets the friction combine mode.
    ///
    /// See the enum ::PxCombineMode .
    ///
    /// Default:
    /// PxCombineMode::eAVERAGE
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setFrictionCombineMode_mut(physx_PxMaterial* self_, int32_t combMode);

    /// Retrieves the friction combine mode.
    ///
    /// See [`setFrictionCombineMode`].
    ///
    /// The friction combine mode for this material.
int32_t      physx_PxMaterial_getFrictionCombineMode(physx_PxMaterial const* self_);

    /// Sets the restitution combine mode.
    ///
    /// See the enum ::PxCombineMode .
    ///
    /// Default:
    /// PxCombineMode::eAVERAGE
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake any actors which may be affected.
void      physx_PxMaterial_setRestitutionCombineMode_mut(physx_PxMaterial* self_, int32_t combMode);

    /// Retrieves the restitution combine mode.
    ///
    /// See [`setRestitutionCombineMode`].
    ///
    /// The coefficient of restitution combine mode for this material.
int32_t      physx_PxMaterial_getRestitutionCombineMode(physx_PxMaterial const* self_);

char const*      physx_PxMaterial_getConcreteTypeName(physx_PxMaterial const* self_);

    /// Construct parameters with default values.
physx_PxDiffuseParticleParams      physx_PxDiffuseParticleParams_new();

    /// (re)sets the structure to the default.
void      physx_PxDiffuseParticleParams_setToDefault_mut(physx_PxDiffuseParticleParams* self_);

    /// Sets friction
void      physx_PxParticleMaterial_setFriction_mut(physx_PxParticleMaterial* self_, float friction);

    /// Retrieves the friction value.
    ///
    /// The friction value.
float      physx_PxParticleMaterial_getFriction(physx_PxParticleMaterial const* self_);

    /// Sets velocity damping term
void      physx_PxParticleMaterial_setDamping_mut(physx_PxParticleMaterial* self_, float damping);

    /// Retrieves the velocity damping term
    ///
    /// The velocity damping term.
float      physx_PxParticleMaterial_getDamping(physx_PxParticleMaterial const* self_);

    /// Sets adhesion term
void      physx_PxParticleMaterial_setAdhesion_mut(physx_PxParticleMaterial* self_, float adhesion);

    /// Retrieves the adhesion term
    ///
    /// The adhesion term.
float      physx_PxParticleMaterial_getAdhesion(physx_PxParticleMaterial const* self_);

    /// Sets gravity scale term
void      physx_PxParticleMaterial_setGravityScale_mut(physx_PxParticleMaterial* self_, float scale);

    /// Retrieves the gravity scale term
    ///
    /// The gravity scale term.
float      physx_PxParticleMaterial_getGravityScale(physx_PxParticleMaterial const* self_);

    /// Sets material adhesion radius scale. This is multiplied by the particle rest offset to compute the fall-off distance
    /// at which point adhesion ceases to operate.
void      physx_PxParticleMaterial_setAdhesionRadiusScale_mut(physx_PxParticleMaterial* self_, float scale);

    /// Retrieves the adhesion radius scale.
    ///
    /// The adhesion radius scale.
float      physx_PxParticleMaterial_getAdhesionRadiusScale(physx_PxParticleMaterial const* self_);

    /// Destroys the instance it is called on.
    ///
    /// Use this release method to destroy an instance of this class. Be sure
    /// to not keep a reference to this object after calling release.
    /// Avoid release calls while a scene is simulating (in between simulate() and fetchResults() calls).
    ///
    /// Note that this must be called once for each prior call to PxCreatePhysics, as
    /// there is a reference counter. Also note that you mustn't destroy the PxFoundation instance (holding the allocator, error callback etc.)
    /// until after the reference count reaches 0 and the SDK is actually removed.
    ///
    /// Releasing an SDK will also release any objects created through it (scenes, triangle meshes, convex meshes, heightfields, shapes etc.),
    /// provided the user hasn't already done so.
    ///
    /// Releasing the PxPhysics instance is a prerequisite to releasing the PxFoundation instance.
void      physx_PxPhysics_release_mut(physx_PxPhysics* self_);

    /// Retrieves the Foundation instance.
    ///
    /// A reference to the Foundation object.
physx_PxFoundation*      physx_PxPhysics_getFoundation_mut(physx_PxPhysics* self_);

    /// Creates an aggregate with the specified maximum size and filtering hint.
    ///
    /// The previous API used "bool enableSelfCollision" which should now silently evaluates
    /// to a PxAggregateType::eGENERIC aggregate with its self-collision bit.
    ///
    /// Use PxAggregateType::eSTATIC or PxAggregateType::eKINEMATIC for aggregates that will
    /// only contain static or kinematic actors. This provides faster filtering when used in
    /// combination with PxPairFilteringMode.
    ///
    /// The new aggregate.
physx_PxAggregate*      physx_PxPhysics_createAggregate_mut(physx_PxPhysics* self_, uint32_t maxActor, uint32_t maxShape, uint32_t filterHint);

    /// Returns the simulation tolerance parameters.
    ///
    /// The current simulation tolerance parameters.
physx_PxTolerancesScale const*      physx_PxPhysics_getTolerancesScale(physx_PxPhysics const* self_);

    /// Creates a triangle mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new triangle mesh.
physx_PxTriangleMesh*      physx_PxPhysics_createTriangleMesh_mut(physx_PxPhysics* self_, physx_PxInputStream* stream);

    /// Return the number of triangle meshes that currently exist.
    ///
    /// Number of triangle meshes.
uint32_t      physx_PxPhysics_getNbTriangleMeshes(physx_PxPhysics const* self_);

    /// Writes the array of triangle mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the triangle meshes in the array is not specified.
    ///
    /// The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getTriangleMeshes(physx_PxPhysics const* self_, physx_PxTriangleMesh** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a tetrahedron mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new tetrahedron mesh.
physx_PxTetrahedronMesh*      physx_PxPhysics_createTetrahedronMesh_mut(physx_PxPhysics* self_, physx_PxInputStream* stream);

    /// Creates a softbody mesh object.
    ///
    /// The new softbody mesh.
physx_PxSoftBodyMesh*      physx_PxPhysics_createSoftBodyMesh_mut(physx_PxPhysics* self_, physx_PxInputStream* stream);

    /// Return the number of tetrahedron meshes that currently exist.
    ///
    /// Number of tetrahedron meshes.
uint32_t      physx_PxPhysics_getNbTetrahedronMeshes(physx_PxPhysics const* self_);

    /// Writes the array of tetrahedron mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the tetrahedron meshes in the array is not specified.
    ///
    /// The number of tetrahedron mesh pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getTetrahedronMeshes(physx_PxPhysics const* self_, physx_PxTetrahedronMesh** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a heightfield object from previously cooked stream.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new heightfield.
physx_PxHeightField*      physx_PxPhysics_createHeightField_mut(physx_PxPhysics* self_, physx_PxInputStream* stream);

    /// Return the number of heightfields that currently exist.
    ///
    /// Number of heightfields.
uint32_t      physx_PxPhysics_getNbHeightFields(physx_PxPhysics const* self_);

    /// Writes the array of heightfield pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the heightfields in the array is not specified.
    ///
    /// The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getHeightFields(physx_PxPhysics const* self_, physx_PxHeightField** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a convex mesh object.
    ///
    /// This can then be instanced into [`PxShape`] objects.
    ///
    /// The new convex mesh.
physx_PxConvexMesh*      physx_PxPhysics_createConvexMesh_mut(physx_PxPhysics* self_, physx_PxInputStream* stream);

    /// Return the number of convex meshes that currently exist.
    ///
    /// Number of convex meshes.
uint32_t      physx_PxPhysics_getNbConvexMeshes(physx_PxPhysics const* self_);

    /// Writes the array of convex mesh pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the convex meshes in the array is not specified.
    ///
    /// The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getConvexMeshes(physx_PxPhysics const* self_, physx_PxConvexMesh** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a bounding volume hierarchy.
    ///
    /// The new BVH.
physx_PxBVH*      physx_PxPhysics_createBVH_mut(physx_PxPhysics* self_, physx_PxInputStream* stream);

    /// Return the number of bounding volume hierarchies that currently exist.
    ///
    /// Number of bounding volume hierarchies.
uint32_t      physx_PxPhysics_getNbBVHs(physx_PxPhysics const* self_);

    /// Writes the array of bounding volume hierarchy pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the BVHs in the array is not specified.
    ///
    /// The number of BVH pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getBVHs(physx_PxPhysics const* self_, physx_PxBVH** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a scene.
    ///
    /// Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
    /// number of scenes that can be created.
    ///
    /// The new scene object.
physx_PxScene*      physx_PxPhysics_createScene_mut(physx_PxPhysics* self_, physx_PxSceneDesc const* sceneDesc);

    /// Gets number of created scenes.
    ///
    /// The number of scenes created.
uint32_t      physx_PxPhysics_getNbScenes(physx_PxPhysics const* self_);

    /// Writes the array of scene pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the scene pointers in the array is not specified.
    ///
    /// The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getScenes(physx_PxPhysics const* self_, physx_PxScene** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a static rigid actor with the specified pose and all other fields initialized
    /// to their default values.
physx_PxRigidStatic*      physx_PxPhysics_createRigidStatic_mut(physx_PxPhysics* self_, physx_PxTransform const* pose);

    /// Creates a dynamic rigid actor with the specified pose and all other fields initialized
    /// to their default values.
physx_PxRigidDynamic*      physx_PxPhysics_createRigidDynamic_mut(physx_PxPhysics* self_, physx_PxTransform const* pose);

    /// Creates a pruning structure from actors.
    ///
    /// Every provided actor needs at least one shape with the eSCENE_QUERY_SHAPE flag set.
    ///
    /// Both static and dynamic actors can be provided.
    ///
    /// It is not allowed to pass in actors which are already part of a scene.
    ///
    /// Articulation links cannot be provided.
    ///
    /// Pruning structure created from given actors, or NULL if any of the actors did not comply with the above requirements.
physx_PxPruningStructure*      physx_PxPhysics_createPruningStructure_mut(physx_PxPhysics* self_, physx_PxRigidActor* const* actors, uint32_t nbActors);

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
physx_PxShape*      physx_PxPhysics_createShape_mut(physx_PxPhysics* self_, physx_PxGeometry const* geometry, physx_PxMaterial const* material, bool isExclusive, uint8_t shapeFlags);

    /// Creates a shape which may be attached to multiple actors
    ///
    /// The shape will be created with a reference count of 1.
    ///
    /// The shape
    ///
    /// Shared shapes are not mutable when they are attached to an actor
    ///
    /// Shapes created from *SDF* triangle-mesh geometries do not support more than one material.
physx_PxShape*      physx_PxPhysics_createShape_mut_1(physx_PxPhysics* self_, physx_PxGeometry const* geometry, physx_PxMaterial* const* materials, uint16_t materialCount, bool isExclusive, uint8_t shapeFlags);

    /// Return the number of shapes that currently exist.
    ///
    /// Number of shapes.
uint32_t      physx_PxPhysics_getNbShapes(physx_PxPhysics const* self_);

    /// Writes the array of shape pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the shapes in the array is not specified.
    ///
    /// The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getShapes(physx_PxPhysics const* self_, physx_PxShape** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Creates a constraint shader.
    ///
    /// A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
    /// be NULL to denote attachment to the world.
    ///
    /// The new constraint shader.
physx_PxConstraint*      physx_PxPhysics_createConstraint_mut(physx_PxPhysics* self_, physx_PxRigidActor* actor0, physx_PxRigidActor* actor1, physx_PxConstraintConnector* connector, physx_PxConstraintShaderTable const* shaders, uint32_t dataSize);

    /// Creates a reduced-coordinate articulation with all fields initialized to their default values.
    ///
    /// the new articulation
physx_PxArticulationReducedCoordinate*      physx_PxPhysics_createArticulationReducedCoordinate_mut(physx_PxPhysics* self_);

    /// Creates a new rigid body material with certain default properties.
    ///
    /// The new rigid body material.
physx_PxMaterial*      physx_PxPhysics_createMaterial_mut(physx_PxPhysics* self_, float staticFriction, float dynamicFriction, float restitution);

    /// Return the number of rigid body materials that currently exist.
    ///
    /// Number of rigid body materials.
uint32_t      physx_PxPhysics_getNbMaterials(physx_PxPhysics const* self_);

    /// Writes the array of rigid body material pointers to a user buffer.
    ///
    /// Returns the number of pointers written.
    ///
    /// The ordering of the materials in the array is not specified.
    ///
    /// The number of material pointers written to userBuffer, this should be less or equal to bufferSize.
uint32_t      physx_PxPhysics_getMaterials(physx_PxPhysics const* self_, physx_PxMaterial** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Register a deletion listener. Listeners will be called whenever an object is deleted.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
    ///
    /// By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use [`registerDeletionListenerObjects`] to restrict the received events to specific objects.
    ///
    /// The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.
void      physx_PxPhysics_registerDeletionListener_mut(physx_PxPhysics* self_, physx_PxDeletionListener* observer, uint8_t const* deletionEvents, bool restrictedObjectSet);

    /// Unregister a deletion listener.
    ///
    /// It is illegal to register or unregister a deletion listener while deletions are being processed.
void      physx_PxPhysics_unregisterDeletionListener_mut(physx_PxPhysics* self_, physx_PxDeletionListener* observer);

    /// Register specific objects for deletion events.
    ///
    /// This method allows for a deletion listener to limit deletion events to specific objects only.
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
void      physx_PxPhysics_registerDeletionListenerObjects_mut(physx_PxPhysics* self_, physx_PxDeletionListener* observer, physx_PxBase const* const* observables, uint32_t observableCount);

    /// Unregister specific objects for deletion events.
    ///
    /// This method allows to clear previously registered objects for a deletion listener (see [`registerDeletionListenerObjects`]()).
    ///
    /// It is illegal to register or unregister objects while deletions are being processed.
    ///
    /// The deletion listener has to be registered through [`registerDeletionListener`]() and configured to support restricted object sets prior to this method being used.
void      physx_PxPhysics_unregisterDeletionListenerObjects_mut(physx_PxPhysics* self_, physx_PxDeletionListener* observer, physx_PxBase const* const* observables, uint32_t observableCount);

    /// Gets PxPhysics object insertion interface.
    ///
    /// The insertion interface is needed for PxCreateTriangleMesh, PxCooking::createTriangleMesh etc., this allows runtime mesh creation.
physx_PxInsertionCallback*      physx_PxPhysics_getPhysicsInsertionCallback_mut(physx_PxPhysics* self_);

    /// Creates an instance of the physics SDK.
    ///
    /// Creates an instance of this class. May not be a class member to avoid name mangling.
    /// Pass the constant [`PX_PHYSICS_VERSION`] as the argument.
    /// There may be only one instance of this class per process. Calling this method after an instance
    /// has been created already will result in an error message and NULL will be returned.
    ///
    /// Calling this will register all optional code modules (Articulations and HeightFields), preparing them for use.
    /// If you do not need some of these modules, consider calling PxCreateBasePhysics() instead and registering needed
    /// modules manually.
    ///
    /// PxPhysics instance on success, NULL if operation failed
physx_PxPhysics*      physx_phys_PxCreatePhysics(uint32_t version, physx_PxFoundation* foundation, physx_PxTolerancesScale const* scale, bool trackOutstandingAllocations, physx_PxPvd* pvd, physx_PxOmniPvd* omniPvd);

physx_PxPhysics*      physx_phys_PxGetPhysics();

physx_PxActorShape      physx_PxActorShape_new();

physx_PxActorShape      physx_PxActorShape_new_1(physx_PxRigidActor* a, physx_PxShape* s);

    /// constructor sets to default
physx_PxQueryCache      physx_PxQueryCache_new();

    /// constructor to set properties
physx_PxQueryCache      physx_PxQueryCache_new_1(physx_PxShape* s, uint32_t findex);

    /// default constructor
physx_PxQueryFilterData      physx_PxQueryFilterData_new();

    /// constructor to set both filter data and filter flags
physx_PxQueryFilterData      physx_PxQueryFilterData_new_1(physx_PxFilterData const* fd, uint16_t f);

    /// constructor to set filter flags only
physx_PxQueryFilterData      physx_PxQueryFilterData_new_2(uint16_t f);

    /// This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.
    ///
    /// the updated type for this hit  (see [`PxQueryHitType`])
int32_t      physx_PxQueryFilterCallback_preFilter_mut(physx_PxQueryFilterCallback* self_, physx_PxFilterData const* filterData, physx_PxShape const* shape, physx_PxRigidActor const* actor, uint16_t* queryFlags);

    /// This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.
    ///
    /// the updated hit type for this hit  (see [`PxQueryHitType`])
int32_t      physx_PxQueryFilterCallback_postFilter_mut(physx_PxQueryFilterCallback* self_, physx_PxFilterData const* filterData, physx_PxQueryHit const* hit, physx_PxShape const* shape, physx_PxRigidActor const* actor);

    /// virtual destructor
void      physx_PxQueryFilterCallback_delete(physx_PxQueryFilterCallback* self_);

    /// Moves kinematically controlled dynamic actors through the game world.
    ///
    /// You set a dynamic actor to be kinematic using the PxRigidBodyFlag::eKINEMATIC flag
    /// with setRigidBodyFlag().
    ///
    /// The move command will result in a velocity that will move the body into
    /// the desired pose. After the move is carried out during a single time step,
    /// the velocity is returned to zero. Thus, you must continuously call
    /// this in every time step for kinematic actors so that they move along a path.
    ///
    /// This function simply stores the move destination until the next simulation
    /// step is processed, so consecutive calls will simply overwrite the stored target variable.
    ///
    /// The motion is always fully carried out.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and will set the wake counter to [`PxSceneDesc::wakeCounterResetValue`].
void      physx_PxRigidDynamic_setKinematicTarget_mut(physx_PxRigidDynamic* self_, physx_PxTransform const* destination);

    /// Get target pose of a kinematically controlled dynamic actor.
    ///
    /// True if the actor is a kinematically controlled dynamic and the target has been set, else False.
bool      physx_PxRigidDynamic_getKinematicTarget(physx_PxRigidDynamic const* self_, physx_PxTransform* target);

    /// Returns true if this body is sleeping.
    ///
    /// When an actor does not move for a period of time, it is no longer simulated in order to save time. This state
    /// is called sleeping. However, because the object automatically wakes up when it is either touched by an awake object,
    /// or one of its properties is changed by the user, the entire sleep mechanism should be transparent to the user.
    ///
    /// In general, a dynamic rigid actor is guaranteed to be awake if at least one of the following holds:
    ///
    /// The wake counter is positive (see [`setWakeCounter`]()).
    ///
    /// The linear or angular velocity is non-zero.
    ///
    /// A non-zero force or torque has been applied.
    ///
    /// If a dynamic rigid actor is sleeping, the following state is guaranteed:
    ///
    /// The wake counter is zero.
    ///
    /// The linear and angular velocity is zero.
    ///
    /// There is no force update pending.
    ///
    /// When an actor gets inserted into a scene, it will be considered asleep if all the points above hold, else it will be treated as awake.
    ///
    /// If an actor is asleep after the call to PxScene::fetchResults() returns, it is guaranteed that the pose of the actor
    /// was not changed. You can use this information to avoid updating the transforms of associated objects.
    ///
    /// A kinematic actor is asleep unless a target pose has been set (in which case it will stay awake until two consecutive
    /// simulation steps without a target pose being set have passed). The wake counter will get set to zero or to the reset value
    /// [`PxSceneDesc::wakeCounterResetValue`] in the case where a target pose has been set to be consistent with the definitions above.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// True if the actor is sleeping.
bool      physx_PxRigidDynamic_isSleeping(physx_PxRigidDynamic const* self_);

    /// Sets the mass-normalized kinetic energy threshold below which an actor may go to sleep.
    ///
    /// Actors whose kinetic energy divided by their mass is below this threshold will be candidates for sleeping.
    ///
    /// Default:
    /// 5e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
void      physx_PxRigidDynamic_setSleepThreshold_mut(physx_PxRigidDynamic* self_, float threshold);

    /// Returns the mass-normalized kinetic energy below which an actor may go to sleep.
    ///
    /// The energy threshold for sleeping.
float      physx_PxRigidDynamic_getSleepThreshold(physx_PxRigidDynamic const* self_);

    /// Sets the mass-normalized kinetic energy threshold below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// This value has no effect if PxSceneFlag::eENABLE_STABILIZATION was not enabled on the PxSceneDesc.
    ///
    /// Default:
    /// 1e-5f * PxTolerancesScale::speed * PxTolerancesScale::speed
void      physx_PxRigidDynamic_setStabilizationThreshold_mut(physx_PxRigidDynamic* self_, float threshold);

    /// Returns the mass-normalized kinetic energy below which an actor may participate in stabilization.
    ///
    /// Actors whose kinetic energy divided by their mass is above this threshold will not participate in stabilization.
    ///
    /// The energy threshold for participating in stabilization.
float      physx_PxRigidDynamic_getStabilizationThreshold(physx_PxRigidDynamic const* self_);

    /// Reads the PxRigidDynamic lock flags.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// The values of the PxRigidDynamicLock flags.
uint8_t      physx_PxRigidDynamic_getRigidDynamicLockFlags(physx_PxRigidDynamic const* self_);

    /// Raises or clears a particular rigid dynamic lock flag.
    ///
    /// See the list of flags [`PxRigidDynamicLockFlag`]
    ///
    /// Default:
    /// no flags are set
void      physx_PxRigidDynamic_setRigidDynamicLockFlag_mut(physx_PxRigidDynamic* self_, int32_t flag, bool value);

void      physx_PxRigidDynamic_setRigidDynamicLockFlags_mut(physx_PxRigidDynamic* self_, uint8_t flags);

    /// Retrieves the linear velocity of an actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The linear velocity of the actor.
physx_PxVec3      physx_PxRigidDynamic_getLinearVelocity(physx_PxRigidDynamic const* self_);

    /// Sets the linear velocity of the actor.
    ///
    /// Note that if you continuously set the velocity of an actor yourself,
    /// forces such as gravity or friction will not be able to manifest themselves, because forces directly
    /// influence only the velocity/momentum of an actor.
    ///
    /// Default:
    /// (0.0, 0.0, 0.0)
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the
    /// new velocity is non-zero.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
void      physx_PxRigidDynamic_setLinearVelocity_mut(physx_PxRigidDynamic* self_, physx_PxVec3 const* linVel, bool autowake);

    /// Retrieves the angular velocity of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running (except during PxScene::collide(),
    /// in PxContactModifyCallback or in contact report callbacks).
    ///
    /// The angular velocity of the actor.
physx_PxVec3      physx_PxRigidDynamic_getAngularVelocity(physx_PxRigidDynamic const* self_);

    /// Sets the angular velocity of the actor.
    ///
    /// Note that if you continuously set the angular velocity of an actor yourself,
    /// forces such as friction will not be able to rotate the actor, because forces directly influence only the velocity/momentum.
    ///
    /// Default:
    /// (0.0, 0.0, 0.0)
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping, and the autowake parameter is true (default) or the
    /// new velocity is non-zero.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
void      physx_PxRigidDynamic_setAngularVelocity_mut(physx_PxRigidDynamic* self_, physx_PxVec3 const* angVel, bool autowake);

    /// Sets the wake counter for the actor.
    ///
    /// The wake counter value determines the minimum amount of time until the body can be put to sleep. Please note
    /// that a body will not be put to sleep if the energy is above the specified threshold (see [`setSleepThreshold`]())
    /// or if other awake bodies are touching it.
    ///
    /// Passing in a positive value will wake the actor up automatically.
    ///
    /// It is invalid to use this method for kinematic actors since the wake counter for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Default:
    /// 0.4 (which corresponds to 20 frames for a time step of 0.02)
void      physx_PxRigidDynamic_setWakeCounter_mut(physx_PxRigidDynamic* self_, float wakeCounterValue);

    /// Returns the wake counter of the actor.
    ///
    /// It is not allowed to use this method while the simulation is running.
    ///
    /// The wake counter of the actor.
float      physx_PxRigidDynamic_getWakeCounter(physx_PxRigidDynamic const* self_);

    /// Wakes up the actor if it is sleeping.
    ///
    /// The actor will get woken up and might cause other touching actors to wake up as well during the next simulation step.
    ///
    /// This will set the wake counter of the actor to the value specified in [`PxSceneDesc::wakeCounterResetValue`].
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
void      physx_PxRigidDynamic_wakeUp_mut(physx_PxRigidDynamic* self_);

    /// Forces the actor to sleep.
    ///
    /// The actor will stay asleep during the next simulation step if not touched by another non-sleeping actor.
    ///
    /// Any applied force will be cleared and the velocity and the wake counter of the actor will be set to 0.
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already or if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// It is invalid to use this method for kinematic actors since the sleep state for kinematics is defined
    /// based on whether a target pose has been set (see the comment in [`isSleeping`]()).
void      physx_PxRigidDynamic_putToSleep_mut(physx_PxRigidDynamic* self_);

    /// Sets the solver iteration counts for the body.
    ///
    /// The solver iteration count determines how accurately joints and contacts are resolved.
    /// If you are having trouble with jointed bodies oscillating and behaving erratically, then
    /// setting a higher position iteration count may improve their stability.
    ///
    /// If intersecting bodies are being depenetrated too violently, increase the number of velocity
    /// iterations. More velocity iterations will drive the relative exit velocity of the intersecting
    /// objects closer to the correct value given the restitution.
    ///
    /// Default:
    /// 4 position iterations, 1 velocity iteration
void      physx_PxRigidDynamic_setSolverIterationCounts_mut(physx_PxRigidDynamic* self_, uint32_t minPositionIters, uint32_t minVelocityIters);

    /// Retrieves the solver iteration counts.
void      physx_PxRigidDynamic_getSolverIterationCounts(physx_PxRigidDynamic const* self_, uint32_t* minPositionIters, uint32_t* minVelocityIters);

    /// Retrieves the force threshold for contact reports.
    ///
    /// The contact report threshold is a force threshold. If the force between
    /// two actors exceeds this threshold for either of the two actors, a contact report
    /// will be generated according to the contact report threshold flags provided by
    /// the filter shader/callback.
    /// See [`PxPairFlag`].
    ///
    /// The threshold used for a collision between a dynamic actor and the static environment is
    /// the threshold of the dynamic actor, and all contacts with static actors are summed to find
    /// the total normal force.
    ///
    /// Default:
    /// PX_MAX_F32
    ///
    /// Force threshold for contact reports.
float      physx_PxRigidDynamic_getContactReportThreshold(physx_PxRigidDynamic const* self_);

    /// Sets the force threshold for contact reports.
    ///
    /// See [`getContactReportThreshold`]().
void      physx_PxRigidDynamic_setContactReportThreshold_mut(physx_PxRigidDynamic* self_, float threshold);

char const*      physx_PxRigidDynamic_getConcreteTypeName(physx_PxRigidDynamic const* self_);

char const*      physx_PxRigidStatic_getConcreteTypeName(physx_PxRigidStatic const* self_);

    /// constructor sets to default.
physx_PxSceneQueryDesc      physx_PxSceneQueryDesc_new();

    /// (re)sets the structure to the default.
void      physx_PxSceneQueryDesc_setToDefault_mut(physx_PxSceneQueryDesc* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
bool      physx_PxSceneQueryDesc_isValid(physx_PxSceneQueryDesc const* self_);

    /// Sets the rebuild rate of the dynamic tree pruning structures.
void      physx_PxSceneQuerySystemBase_setDynamicTreeRebuildRateHint_mut(physx_PxSceneQuerySystemBase* self_, uint32_t dynamicTreeRebuildRateHint);

    /// Retrieves the rebuild rate of the dynamic tree pruning structures.
    ///
    /// The rebuild rate of the dynamic tree pruning structures.
uint32_t      physx_PxSceneQuerySystemBase_getDynamicTreeRebuildRateHint(physx_PxSceneQuerySystemBase const* self_);

    /// Forces dynamic trees to be immediately rebuilt.
    ///
    /// PxScene will call this function with the PX_SCENE_PRUNER_STATIC or PX_SCENE_PRUNER_DYNAMIC value.
void      physx_PxSceneQuerySystemBase_forceRebuildDynamicTree_mut(physx_PxSceneQuerySystemBase* self_, uint32_t prunerIndex);

    /// Sets scene query update mode
void      physx_PxSceneQuerySystemBase_setUpdateMode_mut(physx_PxSceneQuerySystemBase* self_, int32_t updateMode);

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
int32_t      physx_PxSceneQuerySystemBase_getUpdateMode(physx_PxSceneQuerySystemBase const* self_);

    /// Retrieves the system's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
uint32_t      physx_PxSceneQuerySystemBase_getStaticTimestamp(physx_PxSceneQuerySystemBase const* self_);

    /// Flushes any changes to the scene query representation.
    ///
    /// This method updates the state of the scene query representation to match changes in the scene state.
    ///
    /// By default, these changes are buffered until the next query is submitted. Calling this function will not change
    /// the results from scene queries, but can be used to ensure that a query will not perform update work in the course of
    /// its execution.
    ///
    /// A thread performing updates will hold a write lock on the query structure, and thus stall other querying threads. In multithread
    /// scenarios it can be useful to explicitly schedule the period where this lock may be held for a significant period, so that
    /// subsequent queries issued from multiple threads will not block.
void      physx_PxSceneQuerySystemBase_flushUpdates_mut(physx_PxSceneQuerySystemBase* self_);

    /// Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
    /// or via a custom user callback implementation inheriting from PxRaycastCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
bool      physx_PxSceneQuerySystemBase_raycast(physx_PxSceneQuerySystemBase const* self_, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, float distance, physx_PxRaycastCallback* hitCall, uint16_t hitFlags, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, uint32_t queryFlags);

    /// Performs a sweep test against objects in the scene, returns results in a PxSweepBuffer object
    /// or via a custom user callback implementation inheriting from PxSweepCallback.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
bool      physx_PxSceneQuerySystemBase_sweep(physx_PxSceneQuerySystemBase const* self_, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxVec3 const* unitDir, float distance, physx_PxSweepCallback* hitCall, uint16_t hitFlags, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, float inflation, uint32_t queryFlags);

    /// Performs an overlap test of a given geometry against objects in the scene, returns results in a PxOverlapBuffer object
    /// or via a custom user callback implementation inheriting from PxOverlapCallback.
    ///
    /// Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see [`PxQueryHitType`]).
    ///
    /// True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
    ///
    /// eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
    ///
    /// If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
bool      physx_PxSceneQuerySystemBase_overlap(physx_PxSceneQuerySystemBase const* self_, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxOverlapCallback* hitCall, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, uint32_t queryFlags);

    /// Sets scene query update mode
void      physx_PxSceneSQSystem_setSceneQueryUpdateMode_mut(physx_PxSceneSQSystem* self_, int32_t updateMode);

    /// Gets scene query update mode
    ///
    /// Current scene query update mode.
int32_t      physx_PxSceneSQSystem_getSceneQueryUpdateMode(physx_PxSceneSQSystem const* self_);

    /// Retrieves the scene's internal scene query timestamp, increased each time a change to the
    /// static scene query structure is performed.
    ///
    /// scene query static timestamp
uint32_t      physx_PxSceneSQSystem_getSceneQueryStaticTimestamp(physx_PxSceneSQSystem const* self_);

    /// Flushes any changes to the scene query representation.
void      physx_PxSceneSQSystem_flushQueryUpdates_mut(physx_PxSceneSQSystem* self_);

    /// Forces dynamic trees to be immediately rebuilt.
void      physx_PxSceneSQSystem_forceDynamicTreeRebuild_mut(physx_PxSceneSQSystem* self_, bool rebuildStaticStructure, bool rebuildDynamicStructure);

    /// Return the value of PxSceneQueryDesc::staticStructure that was set when creating the scene with PxPhysics::createScene
int32_t      physx_PxSceneSQSystem_getStaticStructure(physx_PxSceneSQSystem const* self_);

    /// Return the value of PxSceneQueryDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene
int32_t      physx_PxSceneSQSystem_getDynamicStructure(physx_PxSceneSQSystem const* self_);

    /// Executes scene queries update tasks.
    ///
    /// This function will refit dirty shapes within the pruner and will execute a task to build a new AABB tree, which is
    /// build on a different thread. The new AABB tree is built based on the dynamic tree rebuild hint rate. Once
    /// the new tree is ready it will be commited in next fetchQueries call, which must be called after.
    ///
    /// This function is equivalent to the following PxSceneQuerySystem calls:
    /// Synchronous calls:
    /// - PxSceneQuerySystemBase::flushUpdates()
    /// - handle0 = PxSceneQuerySystem::prepareSceneQueryBuildStep(PX_SCENE_PRUNER_STATIC)
    /// - handle1 = PxSceneQuerySystem::prepareSceneQueryBuildStep(PX_SCENE_PRUNER_DYNAMIC)
    /// Asynchronous calls:
    /// - PxSceneQuerySystem::sceneQueryBuildStep(handle0);
    /// - PxSceneQuerySystem::sceneQueryBuildStep(handle1);
    ///
    /// This function is part of the PxSceneSQSystem interface because it uses the PxScene task system under the hood. But
    /// it calls PxSceneQuerySystem functions, which are independent from this system and could be called in a similar
    /// fashion by a separate, possibly user-defined task manager.
    ///
    /// If PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED is used, it is required to update the scene queries
    /// using this function.
void      physx_PxSceneSQSystem_sceneQueriesUpdate_mut(physx_PxSceneSQSystem* self_, physx_PxBaseTask* completionTask, bool controlSimulation);

    /// This checks to see if the scene queries update has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
bool      physx_PxSceneSQSystem_checkQueries_mut(physx_PxSceneSQSystem* self_, bool block);

    /// This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call,
    /// the SDK will issue an error message.
    ///
    /// If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree.
bool      physx_PxSceneSQSystem_fetchQueries_mut(physx_PxSceneSQSystem* self_, bool block);

    /// Decrements the reference count of the object and releases it if the new reference count is zero.
void      physx_PxSceneQuerySystem_release_mut(physx_PxSceneQuerySystem* self_);

    /// Acquires a counted reference to this object.
    ///
    /// This method increases the reference count of the object by 1. Decrement the reference count by calling release()
void      physx_PxSceneQuerySystem_acquireReference_mut(physx_PxSceneQuerySystem* self_);

    /// Preallocates internal arrays to minimize the amount of reallocations.
    ///
    /// The system does not prevent more allocations than given numbers. It is legal to not call this function at all,
    /// or to add more shapes to the system than the preallocated amounts.
void      physx_PxSceneQuerySystem_preallocate_mut(physx_PxSceneQuerySystem* self_, uint32_t prunerIndex, uint32_t nbShapes);

    /// Frees internal memory that may not be in-use anymore.
    ///
    /// This is an entry point for reclaiming transient memory allocated at some point by the SQ system,
    /// but which wasn't been immediately freed for performance reason. Calling this function might free
    /// some memory, but it might also produce a new set of allocations in the next frame.
void      physx_PxSceneQuerySystem_flushMemory_mut(physx_PxSceneQuerySystem* self_);

    /// Adds a shape to the SQ system.
    ///
    /// The same function is used to add either a regular shape, or a SQ compound shape.
void      physx_PxSceneQuerySystem_addSQShape_mut(physx_PxSceneQuerySystem* self_, physx_PxRigidActor const* actor, physx_PxShape const* shape, physx_PxBounds3 const* bounds, physx_PxTransform const* transform, uint32_t const* compoundHandle, bool hasPruningStructure);

    /// Removes a shape from the SQ system.
    ///
    /// The same function is used to remove either a regular shape, or a SQ compound shape.
void      physx_PxSceneQuerySystem_removeSQShape_mut(physx_PxSceneQuerySystem* self_, physx_PxRigidActor const* actor, physx_PxShape const* shape);

    /// Updates a shape in the SQ system.
    ///
    /// The same function is used to update either a regular shape, or a SQ compound shape.
    ///
    /// The transforms are eager-evaluated, but the bounds are lazy-evaluated. This means that
    /// the updated transform has to be passed to the update function, while the bounds are automatically
    /// recomputed by the system whenever needed.
void      physx_PxSceneQuerySystem_updateSQShape_mut(physx_PxSceneQuerySystem* self_, physx_PxRigidActor const* actor, physx_PxShape const* shape, physx_PxTransform const* transform);

    /// Adds a compound to the SQ system.
    ///
    /// SQ compound handle
uint32_t      physx_PxSceneQuerySystem_addSQCompound_mut(physx_PxSceneQuerySystem* self_, physx_PxRigidActor const* actor, physx_PxShape const** shapes, physx_PxBVH const* bvh, physx_PxTransform const* transforms);

    /// Removes a compound from the SQ system.
void      physx_PxSceneQuerySystem_removeSQCompound_mut(physx_PxSceneQuerySystem* self_, uint32_t compoundHandle);

    /// Updates a compound in the SQ system.
    ///
    /// The compound structures are immediately updated when the call occurs.
void      physx_PxSceneQuerySystem_updateSQCompound_mut(physx_PxSceneQuerySystem* self_, uint32_t compoundHandle, physx_PxTransform const* compoundTransform);

    /// Shift the data structures' origin by the specified vector.
    ///
    /// Please refer to the notes of the similar function in PxScene.
void      physx_PxSceneQuerySystem_shiftOrigin_mut(physx_PxSceneQuerySystem* self_, physx_PxVec3 const* shift);

    /// Merges a pruning structure with the SQ system's internal pruners.
void      physx_PxSceneQuerySystem_merge_mut(physx_PxSceneQuerySystem* self_, physx_PxPruningStructure const* pruningStructure);

    /// Shape to SQ-pruner-handle mapping function.
    ///
    /// This function finds and returns the SQ pruner handle associated with a given (actor/shape) couple
    /// that was previously added to the system. This is needed for the sync function.
    ///
    /// Associated SQ pruner handle.
uint32_t      physx_PxSceneQuerySystem_getHandle(physx_PxSceneQuerySystem const* self_, physx_PxRigidActor const* actor, physx_PxShape const* shape, uint32_t* prunerIndex);

    /// Synchronizes the scene-query system with another system that references the same objects.
    ///
    /// This function is used when the scene-query objects also exist in another system that can also update them. For example the scene-query objects
    /// (used for raycast, overlap or sweep queries) might be driven by equivalent objects in an external rigid-body simulation engine. In this case
    /// the rigid-body simulation engine computes the new poses and transforms, and passes them to the scene-query system using this function. It is
    /// more efficient than calling updateSQShape on each object individually, since updateSQShape would end up recomputing the bounds already available
    /// in the rigid-body engine.
void      physx_PxSceneQuerySystem_sync_mut(physx_PxSceneQuerySystem* self_, uint32_t prunerIndex, uint32_t const* handles, uint32_t const* indices, physx_PxBounds3 const* bounds, physx_PxTransformPadded const* transforms, uint32_t count, physx_PxBitMap const* ignoredIndices);

    /// Finalizes updates made to the SQ system.
    ///
    /// This function should be called after updates have been made to the SQ system, to fully reflect the changes
    /// inside the internal pruners. In particular it should be called:
    /// - after calls to updateSQShape
    /// - after calls to sync
    ///
    /// This function:
    /// - recomputes bounds of manually updated shapes (i.e. either regular or SQ compound shapes modified by updateSQShape)
    /// - updates dynamic pruners (refit operations)
    /// - incrementally rebuilds AABB-trees
    ///
    /// The amount of work performed in this function depends on PxSceneQueryUpdateMode.
void      physx_PxSceneQuerySystem_finalizeUpdates_mut(physx_PxSceneQuerySystem* self_);

    /// Prepares asynchronous build step.
    ///
    /// This is directly called (synchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function is called to let the system execute any necessary synchronous operation before the
    /// asynchronous sceneQueryBuildStep() function is called.
    ///
    /// If there is any work to do for the specific pruner, the function returns a pruner-specific handle that
    /// will be passed to the corresponding, asynchronous sceneQueryBuildStep function.
    ///
    /// A pruner-specific handle that will be sent to sceneQueryBuildStep if there is any work to do, i.e. to execute the corresponding sceneQueryBuildStep() call.
    ///
    /// Null if there is no work to do, otherwise a pruner-specific handle.
void*      physx_PxSceneQuerySystem_prepareSceneQueryBuildStep_mut(physx_PxSceneQuerySystem* self_, uint32_t prunerIndex);

    /// Executes asynchronous build step.
    ///
    /// This is directly called (asynchronously) by PxSceneSQSystem::sceneQueriesUpdate(). See the comments there.
    ///
    /// This function incrementally builds the internal trees/pruners. It is called asynchronously, i.e. this can be
    /// called from different threads for building multiple trees at the same time.
void      physx_PxSceneQuerySystem_sceneQueryBuildStep_mut(physx_PxSceneQuerySystem* self_, void* handle);

physx_PxBroadPhaseDesc      physx_PxBroadPhaseDesc_new(int32_t type_);

bool      physx_PxBroadPhaseDesc_isValid(physx_PxBroadPhaseDesc const* self_);

    /// Retrieves the filter group for static objects.
    ///
    /// Mark static objects with this group when adding them to the broadphase.
    /// Overlaps between static objects will not be detected. All static objects
    /// should have the same group.
    ///
    /// Filter group for static objects.
uint32_t      physx_phys_PxGetBroadPhaseStaticFilterGroup();

    /// Retrieves a filter group for dynamic objects.
    ///
    /// Mark dynamic objects with this group when adding them to the broadphase.
    /// Each dynamic object must have an ID, and overlaps between dynamic objects that have
    /// the same ID will not be detected. This is useful to dismiss overlaps between shapes
    /// of the same (compound) actor directly within the broadphase.
    ///
    /// Filter group for the object.
uint32_t      physx_phys_PxGetBroadPhaseDynamicFilterGroup(uint32_t id);

    /// Retrieves a filter group for kinematic objects.
    ///
    /// Mark kinematic objects with this group when adding them to the broadphase.
    /// Each kinematic object must have an ID, and overlaps between kinematic objects that have
    /// the same ID will not be detected.
    ///
    /// Filter group for the object.
uint32_t      physx_phys_PxGetBroadPhaseKinematicFilterGroup(uint32_t id);

physx_PxBroadPhaseUpdateData      physx_PxBroadPhaseUpdateData_new(uint32_t const* created, uint32_t nbCreated, uint32_t const* updated, uint32_t nbUpdated, uint32_t const* removed, uint32_t nbRemoved, physx_PxBounds3 const* bounds, uint32_t const* groups, float const* distances, uint32_t capacity);

physx_PxBroadPhaseResults      physx_PxBroadPhaseResults_new();

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
uint32_t      physx_PxBroadPhaseRegions_getNbRegions(physx_PxBroadPhaseRegions const* self_);

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions.
uint32_t      physx_PxBroadPhaseRegions_getRegions(physx_PxBroadPhaseRegions const* self_, physx_PxBroadPhaseRegionInfo* userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Adds a new broad-phase region.
    ///
    /// The total number of regions is limited to PxBroadPhaseCaps::mMaxNbRegions. If that number is exceeded, the call is ignored.
    ///
    /// The newly added region will be automatically populated with already existing objects that touch it, if the
    /// 'populateRegion' parameter is set to true. Otherwise the newly added region will be empty, and it will only be
    /// populated with objects when those objects are added to the simulation, or updated if they already exist.
    ///
    /// Using 'populateRegion=true' has a cost, so it is best to avoid it if possible. In particular it is more efficient
    /// to create the empty regions first (with populateRegion=false) and then add the objects afterwards (rather than
    /// the opposite).
    ///
    /// Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
    /// regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
    /// object leaves all regions, or is created outside of all regions, several things happen:
    /// - collisions get disabled for this object
    /// - the object appears in the getOutOfBoundsObjects() array
    ///
    /// If an out-of-bounds object, whose collisions are disabled, re-enters a valid broadphase region, then collisions
    /// are re-enabled for that object.
    ///
    /// Handle for newly created region, or 0xffffffff in case of failure.
uint32_t      physx_PxBroadPhaseRegions_addRegion_mut(physx_PxBroadPhaseRegions* self_, physx_PxBroadPhaseRegion const* region, bool populateRegion, physx_PxBounds3 const* bounds, float const* distances);

    /// Removes a broad-phase region.
    ///
    /// If the region still contains objects, and if those objects do not overlap any region any more, they are not
    /// automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
    /// is used for each object. Users are responsible for removing the objects from the simulation if this is the
    /// desired behavior.
    ///
    /// If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    ///
    /// True if success
bool      physx_PxBroadPhaseRegions_removeRegion_mut(physx_PxBroadPhaseRegions* self_, uint32_t handle);

uint32_t      physx_PxBroadPhaseRegions_getNbOutOfBoundsObjects(physx_PxBroadPhaseRegions const* self_);

uint32_t const*      physx_PxBroadPhaseRegions_getOutOfBoundsObjects(physx_PxBroadPhaseRegions const* self_);

void      physx_PxBroadPhase_release_mut(physx_PxBroadPhase* self_);

    /// Gets the broadphase type.
    ///
    /// Broadphase type.
int32_t      physx_PxBroadPhase_getType(physx_PxBroadPhase const* self_);

    /// Gets broad-phase caps.
void      physx_PxBroadPhase_getCaps(physx_PxBroadPhase const* self_, physx_PxBroadPhaseCaps* caps);

    /// Retrieves the regions API if applicable.
    ///
    /// For broadphases that do not use explicit user-defined regions, this call returns NULL.
    ///
    /// Region API, or NULL.
physx_PxBroadPhaseRegions*      physx_PxBroadPhase_getRegions_mut(physx_PxBroadPhase* self_);

    /// Retrieves the broadphase allocator.
    ///
    /// User-provided buffers should ideally be allocated with this allocator, for best performance.
    /// This is especially true for the GPU broadphases, whose buffers need to be allocated in CUDA
    /// host memory.
    ///
    /// The broadphase allocator.
physx_PxAllocatorCallback*      physx_PxBroadPhase_getAllocator_mut(physx_PxBroadPhase* self_);

    /// Retrieves the profiler's context ID.
    ///
    /// The context ID.
uint64_t      physx_PxBroadPhase_getContextID(physx_PxBroadPhase const* self_);

    /// Sets a scratch buffer
    ///
    /// Some broadphases might take advantage of a scratch buffer to limit runtime allocations.
    ///
    /// All broadphases still work without providing a scratch buffer, this is an optional function
    /// that can potentially reduce runtime allocations.
void      physx_PxBroadPhase_setScratchBlock_mut(physx_PxBroadPhase* self_, void* scratchBlock, uint32_t size);

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The provided update data describes changes to objects since the last broadphase update.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
void      physx_PxBroadPhase_update_mut(physx_PxBroadPhase* self_, physx_PxBroadPhaseUpdateData const* updateData, physx_PxBaseTask* continuation);

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
void      physx_PxBroadPhase_fetchResults_mut(physx_PxBroadPhase* self_, physx_PxBroadPhaseResults* results);

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
void      physx_PxBroadPhase_update_mut_1(physx_PxBroadPhase* self_, physx_PxBroadPhaseResults* results, physx_PxBroadPhaseUpdateData const* updateData);

    /// Broadphase factory function.
    ///
    /// Use this function to create a new standalone broadphase.
    ///
    /// Newly created broadphase, or NULL
physx_PxBroadPhase*      physx_phys_PxCreateBroadPhase(physx_PxBroadPhaseDesc const* desc);

void      physx_PxAABBManager_release_mut(physx_PxAABBManager* self_);

    /// Retrieves the underlying broadphase.
    ///
    /// The managed broadphase.
physx_PxBroadPhase*      physx_PxAABBManager_getBroadPhase_mut(physx_PxAABBManager* self_);

    /// Retrieves the managed bounds.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object bounds.
physx_PxBounds3 const*      physx_PxAABBManager_getBounds(physx_PxAABBManager const* self_);

    /// Retrieves the managed distances.
    ///
    /// This is needed as input parameters to functions like PxBroadPhaseRegions::addRegion.
    ///
    /// The managed object distances.
float const*      physx_PxAABBManager_getDistances(physx_PxAABBManager const* self_);

    /// Retrieves the managed filter groups.
    ///
    /// The managed object groups.
uint32_t const*      physx_PxAABBManager_getGroups(physx_PxAABBManager const* self_);

    /// Retrieves the managed buffers' capacity.
    ///
    /// Bounds, distances and groups buffers have the same capacity.
    ///
    /// The managed buffers' capacity.
uint32_t      physx_PxAABBManager_getCapacity(physx_PxAABBManager const* self_);

    /// Adds an object to the manager.
    ///
    /// Objects' indices are externally managed, i.e. they must be provided by users (as opposed to handles
    /// that could be returned by this manager). The design allows users to identify an object by a single ID,
    /// and use the same ID in multiple sub-systems.
void      physx_PxAABBManager_addObject_mut(physx_PxAABBManager* self_, uint32_t index, physx_PxBounds3 const* bounds, uint32_t group, float distance);

    /// Removes an object from the manager.
void      physx_PxAABBManager_removeObject_mut(physx_PxAABBManager* self_, uint32_t index);

    /// Updates an object in the manager.
    ///
    /// This call can update an object's bounds, distance, or both.
    /// It is not possible to update an object's filter group.
void      physx_PxAABBManager_updateObject_mut(physx_PxAABBManager* self_, uint32_t index, physx_PxBounds3 const* bounds, float const* distance);

    /// Updates the broadphase and computes the lists of created/deleted pairs.
    ///
    /// The data necessary for updating the broadphase is internally computed by the AABB manager.
    ///
    /// To benefit from potentially multithreaded implementations, it is necessary to provide a continuation
    /// task to the function. It is legal to pass NULL there, but the underlying (CPU) implementations will
    /// then run single-threaded.
void      physx_PxAABBManager_update_mut(physx_PxAABBManager* self_, physx_PxBaseTask* continuation);

    /// Retrieves the broadphase results after an update.
    ///
    /// This should be called once after each update call to retrieve the results of the broadphase. The
    /// results are incremental, i.e. the system only returns new and lost pairs, not all current pairs.
void      physx_PxAABBManager_fetchResults_mut(physx_PxAABBManager* self_, physx_PxBroadPhaseResults* results);

    /// Helper for single-threaded updates.
    ///
    /// This short helper function performs a single-theaded update and reports the results in a single call.
void      physx_PxAABBManager_update_mut_1(physx_PxAABBManager* self_, physx_PxBroadPhaseResults* results);

    /// AABB manager factory function.
    ///
    /// Use this function to create a new standalone high-level broadphase.
    ///
    /// Newly created AABB manager, or NULL
physx_PxAABBManager*      physx_phys_PxCreateAABBManager(physx_PxBroadPhase* broadphase);

    /// constructor sets to default
physx_PxSceneLimits      physx_PxSceneLimits_new();

    /// (re)sets the structure to the default
void      physx_PxSceneLimits_setToDefault_mut(physx_PxSceneLimits* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
bool      physx_PxSceneLimits_isValid(physx_PxSceneLimits const* self_);

physx_PxgDynamicsMemoryConfig      physx_PxgDynamicsMemoryConfig_new();

bool      physx_PxgDynamicsMemoryConfig_isValid(physx_PxgDynamicsMemoryConfig const* self_);

    /// constructor sets to default.
physx_PxSceneDesc      physx_PxSceneDesc_new(physx_PxTolerancesScale const* scale);

    /// (re)sets the structure to the default.
void      physx_PxSceneDesc_setToDefault_mut(physx_PxSceneDesc* self_, physx_PxTolerancesScale const* scale);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
bool      physx_PxSceneDesc_isValid(physx_PxSceneDesc const* self_);

physx_PxTolerancesScale const*      physx_PxSceneDesc_getTolerancesScale(physx_PxSceneDesc const* self_);

    /// Get number of broadphase volumes added for the current simulation step.
    ///
    /// Number of broadphase volumes added.
uint32_t      physx_PxSimulationStatistics_getNbBroadPhaseAdds(physx_PxSimulationStatistics const* self_);

    /// Get number of broadphase volumes removed for the current simulation step.
    ///
    /// Number of broadphase volumes removed.
uint32_t      physx_PxSimulationStatistics_getNbBroadPhaseRemoves(physx_PxSimulationStatistics const* self_);

    /// Get number of shape collision pairs of a certain type processed for the current simulation step.
    ///
    /// There is an entry for each geometry pair type.
    ///
    /// entry[i][j] = entry[j][i], hence, if you want the sum of all pair
    /// types, you need to discard the symmetric entries
    ///
    /// Number of processed pairs of the specified geometry types.
uint32_t      physx_PxSimulationStatistics_getRbPairStats(physx_PxSimulationStatistics const* self_, int32_t pairType, int32_t g0, int32_t g1);

physx_PxSimulationStatistics      physx_PxSimulationStatistics_new();

    /// Sets the PVD flag. See PxPvdSceneFlag.
void      physx_PxPvdSceneClient_setScenePvdFlag_mut(physx_PxPvdSceneClient* self_, int32_t flag, bool value);

    /// Sets the PVD flags. See PxPvdSceneFlags.
void      physx_PxPvdSceneClient_setScenePvdFlags_mut(physx_PxPvdSceneClient* self_, uint8_t flags);

    /// Retrieves the PVD flags. See PxPvdSceneFlags.
uint8_t      physx_PxPvdSceneClient_getScenePvdFlags(physx_PxPvdSceneClient const* self_);

    /// update camera on PVD application's render window
void      physx_PxPvdSceneClient_updateCamera_mut(physx_PxPvdSceneClient* self_, char const* name, physx_PxVec3 const* origin, physx_PxVec3 const* up, physx_PxVec3 const* target);

    /// draw points on PVD application's render window
void      physx_PxPvdSceneClient_drawPoints_mut(physx_PxPvdSceneClient* self_, physx_PxDebugPoint const* points, uint32_t count);

    /// draw lines on PVD application's render window
void      physx_PxPvdSceneClient_drawLines_mut(physx_PxPvdSceneClient* self_, physx_PxDebugLine const* lines, uint32_t count);

    /// draw triangles on PVD application's render window
void      physx_PxPvdSceneClient_drawTriangles_mut(physx_PxPvdSceneClient* self_, physx_PxDebugTriangle const* triangles, uint32_t count);

    /// draw text on PVD application's render window
void      physx_PxPvdSceneClient_drawText_mut(physx_PxPvdSceneClient* self_, physx_PxDebugText const* text);

physx_PxDominanceGroupPair      physx_PxDominanceGroupPair_new(uint8_t a, uint8_t b);

void      physx_PxBroadPhaseCallback_delete(physx_PxBroadPhaseCallback* self_);

    /// Out-of-bounds notification.
    ///
    /// This function is called when an object leaves the broad-phase.
void      physx_PxBroadPhaseCallback_onObjectOutOfBounds_mut(physx_PxBroadPhaseCallback* self_, physx_PxShape* shape, physx_PxActor* actor);

    /// Out-of-bounds notification.
    ///
    /// This function is called when an aggregate leaves the broad-phase.
void      physx_PxBroadPhaseCallback_onObjectOutOfBounds_mut_1(physx_PxBroadPhaseCallback* self_, physx_PxAggregate* aggregate);

    /// Deletes the scene.
    ///
    /// Removes any actors and constraint shaders from this scene
    /// (if the user hasn't already done so).
    ///
    /// Be sure to not keep a reference to this object after calling release.
    /// Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
void      physx_PxScene_release_mut(physx_PxScene* self_);

    /// Sets a scene flag. You can only set one flag at a time.
    ///
    /// Not all flags are mutable and changing some will result in an error. Please check [`PxSceneFlag`] to see which flags can be changed.
void      physx_PxScene_setFlag_mut(physx_PxScene* self_, int32_t flag, bool value);

    /// Get the scene flags.
    ///
    /// The scene flags. See [`PxSceneFlag`]
uint32_t      physx_PxScene_getFlags(physx_PxScene const* self_);

    /// Set new scene limits.
    ///
    /// Increase the maximum capacity of various data structures in the scene. The new capacities will be
    /// at least as large as required to deal with the objects currently in the scene. Further, these values
    /// are for preallocation and do not represent hard limits.
void      physx_PxScene_setLimits_mut(physx_PxScene* self_, physx_PxSceneLimits const* limits);

    /// Get current scene limits.
    ///
    /// Current scene limits.
physx_PxSceneLimits      physx_PxScene_getLimits(physx_PxScene const* self_);

    /// Call this method to retrieve the Physics SDK.
    ///
    /// The physics SDK this scene is associated with.
physx_PxPhysics*      physx_PxScene_getPhysics_mut(physx_PxScene* self_);

    /// Retrieves the scene's internal timestamp, increased each time a simulation step is completed.
    ///
    /// scene timestamp
uint32_t      physx_PxScene_getTimestamp(physx_PxScene const* self_);

    /// Adds an articulation to this scene.
    ///
    /// If the articulation is already assigned to a scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// True if success
bool      physx_PxScene_addArticulation_mut(physx_PxScene* self_, physx_PxArticulationReducedCoordinate* articulation);

    /// Removes an articulation from this scene.
    ///
    /// If the articulation is not part of this scene (see [`PxArticulationReducedCoordinate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the articulation is in an aggregate it will be removed from the aggregate.
void      physx_PxScene_removeArticulation_mut(physx_PxScene* self_, physx_PxArticulationReducedCoordinate* articulation, bool wakeOnLostTouch);

    /// Adds an actor to this scene.
    ///
    /// If the actor is already assigned to a scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the actor has an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// You can not add individual articulation links (see [`PxArticulationLink`]) to the scene. Use #addArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// When a BVH is provided the actor shapes are grouped together.
    /// The scene query pruning structure inside PhysX SDK will store/update one
    /// bound per actor. The scene queries against such an actor will query actor
    /// bounds and then make a local space query against the provided BVH, which is in actor's local space.
    ///
    /// True if success
bool      physx_PxScene_addActor_mut(physx_PxScene* self_, physx_PxActor* actor, physx_PxBVH const* bvh);

    /// Adds actors to this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use addActor() instead.
    /// For articulation links, use addArticulation().
    ///
    /// If one of the actors is already assigned to a scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// If an actor in the array contains an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If an actor in the array is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// this method is optimized for high performance.
    ///
    /// True if success
bool      physx_PxScene_addActors_mut(physx_PxScene* self_, physx_PxActor* const* actors, uint32_t nbActors);

    /// Adds a pruning structure together with its actors to this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use addActor() instead.
    /// For articulation links, use addArticulation().
    ///
    /// If an actor in the pruning structure contains an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// For all actors in the pruning structure each assigned PxConstraint object will get added to the scene automatically if
    /// it connects to another actor that is part of the scene already.
    ///
    /// This method is optimized for high performance.
    ///
    /// Merging a PxPruningStructure into an active scene query optimization AABB tree might unbalance the tree. A typical use case for
    /// PxPruningStructure is a large world scenario where blocks of closely positioned actors get streamed in. The merge process finds the
    /// best node in the active scene query optimization AABB tree and inserts the PxPruningStructure. Therefore using PxPruningStructure
    /// for actors scattered throughout the world will result in an unbalanced tree.
    ///
    /// True if success
bool      physx_PxScene_addActors_mut_1(physx_PxScene* self_, physx_PxPruningStructure const* pruningStructure);

    /// Removes an actor from this scene.
    ///
    /// If the actor is not part of this scene (see [`PxActor::getScene`]), the call is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
    ///
    /// If the actor is in an aggregate it will be removed from the aggregate.
void      physx_PxScene_removeActor_mut(physx_PxScene* self_, physx_PxActor* actor, bool wakeOnLostTouch);

    /// Removes actors from this scene. Only supports actors of type PxRigidStatic and PxRigidDynamic.
    ///
    /// This method only supports actors of type PxRigidStatic and PxRigidDynamic. For other actors, use removeActor() instead.
    /// For articulation links, use removeArticulation().
    ///
    /// If some actor is not part of this scene (see [`PxActor::getScene`]), the actor remove is ignored and an error is issued.
    ///
    /// You can not remove individual articulation links (see [`PxArticulationLink`]) from the scene. Use #removeArticulation() instead.
    ///
    /// If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.
void      physx_PxScene_removeActors_mut(physx_PxScene* self_, physx_PxActor* const* actors, uint32_t nbActors, bool wakeOnLostTouch);

    /// Adds an aggregate to this scene.
    ///
    /// If the aggregate is already assigned to a scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// If the aggregate already contains actors, those actors are added to the scene as well.
    ///
    /// True if success
bool      physx_PxScene_addAggregate_mut(physx_PxScene* self_, physx_PxAggregate* aggregate);

    /// Removes an aggregate from this scene.
    ///
    /// If the aggregate is not part of this scene (see [`PxAggregate::getScene`]), the call is ignored and an error is issued.
    ///
    /// If the aggregate contains actors, those actors are removed from the scene as well.
void      physx_PxScene_removeAggregate_mut(physx_PxScene* self_, physx_PxAggregate* aggregate, bool wakeOnLostTouch);

    /// Adds objects in the collection to this scene.
    ///
    /// This function adds the following types of objects to this scene: PxRigidActor (except PxArticulationLink), PxAggregate, PxArticulationReducedCoordinate.
    /// This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.
    ///
    /// If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.
    ///
    /// True if success
bool      physx_PxScene_addCollection_mut(physx_PxScene* self_, physx_PxCollection const* collection);

    /// Retrieve the number of actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// the number of actors.
uint32_t      physx_PxScene_getNbActors(physx_PxScene const* self_, uint16_t types);

    /// Retrieve an array of all the actors of certain types in the scene. For supported types, see PxActorTypeFlags.
    ///
    /// Number of actors written to the buffer.
uint32_t      physx_PxScene_getActors(physx_PxScene const* self_, uint16_t types, physx_PxActor** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Queries the PxScene for a list of the PxActors whose transforms have been
    /// updated during the previous simulation step. Only includes actors of type PxRigidDynamic and PxArticulationLink.
    ///
    /// PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.
    ///
    /// A pointer to the list of active PxActors generated during the last call to fetchResults().
physx_PxActor**      physx_PxScene_getActiveActors_mut(physx_PxScene* self_, uint32_t* nbActorsOut);

    /// Returns the number of articulations in the scene.
    ///
    /// the number of articulations in this scene.
uint32_t      physx_PxScene_getNbArticulations(physx_PxScene const* self_);

    /// Retrieve all the articulations in the scene.
    ///
    /// Number of articulations written to the buffer.
uint32_t      physx_PxScene_getArticulations(physx_PxScene const* self_, physx_PxArticulationReducedCoordinate** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of constraint shaders in the scene.
    ///
    /// the number of constraint shaders in this scene.
uint32_t      physx_PxScene_getNbConstraints(physx_PxScene const* self_);

    /// Retrieve all the constraint shaders in the scene.
    ///
    /// Number of constraint shaders written to the buffer.
uint32_t      physx_PxScene_getConstraints(physx_PxScene const* self_, physx_PxConstraint** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of aggregates in the scene.
    ///
    /// the number of aggregates in this scene.
uint32_t      physx_PxScene_getNbAggregates(physx_PxScene const* self_);

    /// Retrieve all the aggregates in the scene.
    ///
    /// Number of aggregates written to the buffer.
uint32_t      physx_PxScene_getAggregates(physx_PxScene const* self_, physx_PxAggregate** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Specifies the dominance behavior of contacts between two actors with two certain dominance groups.
    ///
    /// It is possible to assign each actor to a dominance groups using [`PxActor::setDominanceGroup`]().
    ///
    /// With dominance groups one can have all contacts created between actors act in one direction only. This is useful, for example, if you
    /// want an object to push debris out of its way and be unaffected,while still responding physically to forces and collisions
    /// with non-debris objects.
    ///
    /// Whenever a contact between two actors (a0, a1) needs to be solved, the groups (g0, g1) of both
    /// actors are retrieved. Then the PxDominanceGroupPair setting for this group pair is retrieved with getDominanceGroupPair(g0, g1).
    ///
    /// In the contact, PxDominanceGroupPair::dominance0 becomes the dominance setting for a0, and
    /// PxDominanceGroupPair::dominance1 becomes the dominance setting for a1. A dominanceN setting of 1.0f, the default,
    /// will permit aN to be pushed or pulled by a(1-N) through the contact. A dominanceN setting of 0.0f, will however
    /// prevent aN to be pushed by a(1-N) via the contact. Thus, a PxDominanceGroupPair of (1.0f, 0.0f) makes
    /// the interaction one-way.
    ///
    /// The matrix sampled by getDominanceGroupPair(g1, g2) is initialised by default such that:
    ///
    /// if g1 == g2, then (1.0f, 1.0f) is returned
    /// if g1
    /// <
    /// g2, then (0.0f, 1.0f) is returned
    /// if g1 >  g2, then (1.0f, 0.0f) is returned
    ///
    /// In other words, we permit actors in higher groups to be pushed around by actors in lower groups by default.
    ///
    /// These settings should cover most applications, and in fact not overriding these settings may likely result in higher performance.
    ///
    /// It is not possible to make the matrix asymetric, or to change the diagonal. In other words:
    ///
    /// it is not possible to change (g1, g2) if (g1==g2)
    /// if you set
    ///
    /// (g1, g2) to X, then (g2, g1) will implicitly and automatically be set to ~X, where:
    ///
    /// ~(1.0f, 1.0f) is (1.0f, 1.0f)
    /// ~(0.0f, 1.0f) is (1.0f, 0.0f)
    /// ~(1.0f, 0.0f) is (0.0f, 1.0f)
    ///
    /// These two restrictions are to make sure that contacts between two actors will always evaluate to the same dominance
    /// setting, regardless of the order of the actors.
    ///
    /// Dominance settings are currently specified as floats 0.0f or 1.0f because in the future we may permit arbitrary
    /// fractional settings to express 'partly-one-way' interactions.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake actors up automatically.
void      physx_PxScene_setDominanceGroupPair_mut(physx_PxScene* self_, uint8_t group1, uint8_t group2, physx_PxDominanceGroupPair const* dominance);

    /// Samples the dominance matrix.
physx_PxDominanceGroupPair      physx_PxScene_getDominanceGroupPair(physx_PxScene const* self_, uint8_t group1, uint8_t group2);

    /// Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene
physx_PxCpuDispatcher*      physx_PxScene_getCpuDispatcher(physx_PxScene const* self_);

    /// Reserves a new client ID.
    ///
    /// PX_DEFAULT_CLIENT is always available as the default clientID.
    /// Additional clients are returned by this function. Clients cannot be released once created.
    /// An error is reported when more than a supported number of clients (currently 128) are created.
uint8_t      physx_PxScene_createClient_mut(physx_PxScene* self_);

    /// Sets a user notify object which receives special simulation events when they occur.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
void      physx_PxScene_setSimulationEventCallback_mut(physx_PxScene* self_, physx_PxSimulationEventCallback* callback);

    /// Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().
    ///
    /// The current user notify pointer. See [`PxSimulationEventCallback`].
physx_PxSimulationEventCallback*      physx_PxScene_getSimulationEventCallback(physx_PxScene const* self_);

    /// Sets a user callback object, which receives callbacks on all contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
void      physx_PxScene_setContactModifyCallback_mut(physx_PxScene* self_, physx_PxContactModifyCallback* callback);

    /// Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
void      physx_PxScene_setCCDContactModifyCallback_mut(physx_PxScene* self_, physx_PxCCDContactModifyCallback* callback);

    /// Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
physx_PxContactModifyCallback*      physx_PxScene_getContactModifyCallback(physx_PxScene const* self_);

    /// Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().
    ///
    /// The current user contact modify callback pointer. See [`PxContactModifyCallback`].
physx_PxCCDContactModifyCallback*      physx_PxScene_getCCDContactModifyCallback(physx_PxScene const* self_);

    /// Sets a broad-phase user callback object.
    ///
    /// Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.
void      physx_PxScene_setBroadPhaseCallback_mut(physx_PxScene* self_, physx_PxBroadPhaseCallback* callback);

    /// Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().
    ///
    /// The current broad-phase callback pointer. See [`PxBroadPhaseCallback`].
physx_PxBroadPhaseCallback*      physx_PxScene_getBroadPhaseCallback(physx_PxScene const* self_);

    /// Sets the shared global filter data which will get passed into the filter shader.
    ///
    /// It is the user's responsibility to ensure that changing the shared global filter data does not change the filter output value for existing pairs.
    /// If the filter output for existing pairs does change nonetheless then such a change will not take effect until the pair gets refiltered.
    /// resetFiltering() can be used to explicitly refilter the pairs of specific objects.
    ///
    /// The provided data will get copied to internal buffers and this copy will be used for filtering calls.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
void      physx_PxScene_setFilterShaderData_mut(physx_PxScene* self_, void const* data, uint32_t dataSize);

    /// Gets the shared global filter data in use for this scene.
    ///
    /// The reference points to a copy of the original filter data specified in [`PxSceneDesc`].filterShaderData or provided by #setFilterShaderData().
    ///
    /// Shared filter data for filter shader.
void const*      physx_PxScene_getFilterShaderData(physx_PxScene const* self_);

    /// Gets the size of the shared global filter data ([`PxSceneDesc`].filterShaderData)
    ///
    /// Size of shared filter data [bytes].
uint32_t      physx_PxScene_getFilterShaderDataSize(physx_PxScene const* self_);

    /// Marks the object to reset interactions and re-run collision filters in the next simulation step.
    ///
    /// This call forces the object to remove all existing collision interactions, to search anew for existing contact
    /// pairs and to run the collision filters again for found collision pairs.
    ///
    /// The operation is supported for PxRigidActor objects only.
    ///
    /// All persistent state of existing interactions will be lost and can not be retrieved even if the same collison pair
    /// is found again in the next step. This will mean, for example, that you will not get notified about persistent contact
    /// for such an interaction (see [`PxPairFlag::eNOTIFY_TOUCH_PERSISTS`]), the contact pair will be interpreted as newly found instead.
    ///
    /// Lost touch contact reports will be sent for every collision pair which includes this shape, if they have
    /// been requested through [`PxPairFlag::eNOTIFY_TOUCH_LOST`] or #PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST.
    ///
    /// This is an expensive operation, don't use it if you don't have to.
    ///
    /// Can be used to retrieve collision pairs that were killed by the collision filters (see [`PxFilterFlag::eKILL`])
    ///
    /// It is invalid to use this method if the actor has not been added to a scene already.
    ///
    /// It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does wake up the actor.
    ///
    /// True if success
bool      physx_PxScene_resetFiltering_mut(physx_PxScene* self_, physx_PxActor* actor);

    /// Marks the object to reset interactions and re-run collision filters for specified shapes in the next simulation step.
    ///
    /// This is a specialization of the resetFiltering(PxActor
    /// &
    /// actor) method and allows to reset interactions for specific shapes of
    /// a PxRigidActor.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does wake up the actor.
bool      physx_PxScene_resetFiltering_mut_1(physx_PxScene* self_, physx_PxRigidActor* actor, physx_PxShape* const* shapes, uint32_t shapeCount);

    /// Gets the pair filtering mode for kinematic-kinematic pairs.
    ///
    /// Filtering mode for kinematic-kinematic pairs.
int32_t      physx_PxScene_getKinematicKinematicFilteringMode(physx_PxScene const* self_);

    /// Gets the pair filtering mode for static-kinematic pairs.
    ///
    /// Filtering mode for static-kinematic pairs.
int32_t      physx_PxScene_getStaticKinematicFilteringMode(physx_PxScene const* self_);

    /// Advances the simulation by an elapsedTime time.
    ///
    /// Large elapsedTime values can lead to instabilities. In such cases elapsedTime
    /// should be subdivided into smaller time intervals and simulate() should be called
    /// multiple times for each interval.
    ///
    /// Calls to simulate() should pair with calls to fetchResults():
    /// Each fetchResults() invocation corresponds to exactly one simulate()
    /// invocation; calling simulate() twice without an intervening fetchResults()
    /// or fetchResults() twice without an intervening simulate() causes an error
    /// condition.
    ///
    /// scene->simulate();
    /// ...do some processing until physics is computed...
    /// scene->fetchResults();
    /// ...now results of run may be retrieved.
    ///
    /// True if success
bool      physx_PxScene_simulate_mut(physx_PxScene* self_, float elapsedTime, physx_PxBaseTask* completionTask, void* scratchMemBlock, uint32_t scratchMemBlockSize, bool controlSimulation);

    /// Performs dynamics phase of the simulation pipeline.
    ///
    /// Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.
    ///
    /// True if success
bool      physx_PxScene_advance_mut(physx_PxScene* self_, physx_PxBaseTask* completionTask);

    /// Performs collision detection for the scene over elapsedTime
    ///
    /// Calls to collide() should be the first method called to simulate a frame.
    ///
    /// True if success
bool      physx_PxScene_collide_mut(physx_PxScene* self_, float elapsedTime, physx_PxBaseTask* completionTask, void* scratchMemBlock, uint32_t scratchMemBlockSize, bool controlSimulation);

    /// This checks to see if the simulation run has completed.
    ///
    /// This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
    /// The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
    ///
    /// True if the results are available.
bool      physx_PxScene_checkResults_mut(physx_PxScene* self_, bool block);

    /// This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error
    /// message.
bool      physx_PxScene_fetchCollision_mut(physx_PxScene* self_, bool block);

    /// This is the big brother to checkResults() it basically does the following:
    ///
    /// True if the results have been fetched.
bool      physx_PxScene_fetchResults_mut(physx_PxScene* self_, bool block, uint32_t* errorState);

    /// This call performs the first section of fetchResults, and returns a pointer to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
    /// for fetchResults() performance.
    ///
    /// After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
    /// permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().
    ///
    /// True if the results have been fetched.
bool      physx_PxScene_fetchResultsStart_mut(physx_PxScene* self_, physx_PxContactPairHeader const** contactPairs, uint32_t* nbContactPairs, bool block);

    /// This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.
    ///
    /// This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with
    /// fetchResultsStart(...) and fetchResultsFinish(...)
void      physx_PxScene_processCallbacks_mut(physx_PxScene* self_, physx_PxBaseTask* continuation);

    /// This call performs the second section of fetchResults.
    ///
    /// It must be called after fetchResultsStart() returns and contact reports have been processed.
    ///
    /// Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.
void      physx_PxScene_fetchResultsFinish_mut(physx_PxScene* self_, uint32_t* errorState);

    /// This call performs the synchronization of particle system data copies.
void      physx_PxScene_fetchResultsParticleSystem_mut(physx_PxScene* self_);

    /// Clear internal buffers and free memory.
    ///
    /// This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
    /// the physics data gets streamed in and a checkpoint with a clean state should be created.
    ///
    /// It is not allowed to call this method while the simulation is running. The call will fail.
void      physx_PxScene_flushSimulation_mut(physx_PxScene* self_, bool sendPendingReports);

    /// Sets a constant gravity for the entire scene.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
void      physx_PxScene_setGravity_mut(physx_PxScene* self_, physx_PxVec3 const* vec);

    /// Retrieves the current gravity setting.
    ///
    /// The current gravity for the scene.
physx_PxVec3      physx_PxScene_getGravity(physx_PxScene const* self_);

    /// Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setBounceThresholdVelocity_mut(physx_PxScene* self_, float t);

    /// Return the bounce threshold velocity.
float      physx_PxScene_getBounceThresholdVelocity(physx_PxScene const* self_);

    /// Sets the maximum number of CCD passes
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setCCDMaxPasses_mut(physx_PxScene* self_, uint32_t ccdMaxPasses);

    /// Gets the maximum number of CCD passes.
    ///
    /// The maximum number of CCD passes.
uint32_t      physx_PxScene_getCCDMaxPasses(physx_PxScene const* self_);

    /// Set the maximum CCD separation.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setCCDMaxSeparation_mut(physx_PxScene* self_, float t);

    /// Gets the maximum CCD separation.
    ///
    /// The maximum CCD separation.
float      physx_PxScene_getCCDMaxSeparation(physx_PxScene const* self_);

    /// Set the CCD threshold.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setCCDThreshold_mut(physx_PxScene* self_, float t);

    /// Gets the CCD threshold.
    ///
    /// The CCD threshold.
float      physx_PxScene_getCCDThreshold(physx_PxScene const* self_);

    /// Set the max bias coefficient.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setMaxBiasCoefficient_mut(physx_PxScene* self_, float t);

    /// Gets the max bias coefficient.
    ///
    /// The max bias coefficient.
float      physx_PxScene_getMaxBiasCoefficient(physx_PxScene const* self_);

    /// Set the friction offset threshold.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setFrictionOffsetThreshold_mut(physx_PxScene* self_, float t);

    /// Gets the friction offset threshold.
float      physx_PxScene_getFrictionOffsetThreshold(physx_PxScene const* self_);

    /// Set the friction correlation distance.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setFrictionCorrelationDistance_mut(physx_PxScene* self_, float t);

    /// Gets the friction correlation distance.
float      physx_PxScene_getFrictionCorrelationDistance(physx_PxScene const* self_);

    /// Return the friction model.
int32_t      physx_PxScene_getFrictionType(physx_PxScene const* self_);

    /// Return the solver model.
int32_t      physx_PxScene_getSolverType(physx_PxScene const* self_);

    /// Function that lets you set debug visualization parameters.
    ///
    /// Returns false if the value passed is out of range for usage specified by the enum.
    ///
    /// Do not use this method while the simulation is running.
    ///
    /// False if the parameter is out of range.
bool      physx_PxScene_setVisualizationParameter_mut(physx_PxScene* self_, int32_t param, float value);

    /// Function that lets you query debug visualization parameters.
    ///
    /// The value of the parameter.
float      physx_PxScene_getVisualizationParameter(physx_PxScene const* self_, int32_t paramEnum);

    /// Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setVisualizationCullingBox_mut(physx_PxScene* self_, physx_PxBounds3 const* box_);

    /// Retrieves the visualization culling box.
    ///
    /// the box to which the geometry will be culled.
physx_PxBounds3      physx_PxScene_getVisualizationCullingBox(physx_PxScene const* self_);

    /// Retrieves the render buffer.
    ///
    /// This will contain the results of any active visualization for this scene.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will result in undefined behaviour.
    ///
    /// The render buffer.
physx_PxRenderBuffer const*      physx_PxScene_getRenderBuffer_mut(physx_PxScene* self_);

    /// Call this method to retrieve statistics for the current simulation step.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
void      physx_PxScene_getSimulationStatistics(physx_PxScene const* self_, physx_PxSimulationStatistics* stats);

    /// Returns broad-phase type.
    ///
    /// Broad-phase type
int32_t      physx_PxScene_getBroadPhaseType(physx_PxScene const* self_);

    /// Gets broad-phase caps.
    ///
    /// True if success
bool      physx_PxScene_getBroadPhaseCaps(physx_PxScene const* self_, physx_PxBroadPhaseCaps* caps);

    /// Returns number of regions currently registered in the broad-phase.
    ///
    /// Number of regions
uint32_t      physx_PxScene_getNbBroadPhaseRegions(physx_PxScene const* self_);

    /// Gets broad-phase regions.
    ///
    /// Number of written out regions
uint32_t      physx_PxScene_getBroadPhaseRegions(physx_PxScene const* self_, physx_PxBroadPhaseRegionInfo* userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Adds a new broad-phase region.
    ///
    /// The bounds for the new region must be non-empty, otherwise an error occurs and the call is ignored.
    ///
    /// Note that by default, objects already existing in the SDK that might touch this region will not be automatically
    /// added to the region. In other words the newly created region will be empty, and will only be populated with new
    /// objects when they are added to the simulation, or with already existing objects when they are updated.
    ///
    /// It is nonetheless possible to override this default behavior and let the SDK populate the new region automatically
    /// with already existing objects overlapping the incoming region. This has a cost though, and it should only be used
    /// when the game can not guarantee that all objects within the new region will be added to the simulation after the
    /// region itself.
    ///
    /// Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
    /// regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
    /// object leaves all regions, or is created outside of all regions, several things happen:
    /// - collisions get disabled for this object
    /// - if a PxBroadPhaseCallback object is provided, an "out-of-bounds" event is generated via that callback
    /// - if a PxBroadPhaseCallback object is not provided, a warning/error message is sent to the error stream
    ///
    /// If an object goes out-of-bounds and user deletes it during the same frame, neither the out-of-bounds event nor the
    /// error message is generated.
    ///
    /// Handle for newly created region, or 0xffffffff in case of failure.
uint32_t      physx_PxScene_addBroadPhaseRegion_mut(physx_PxScene* self_, physx_PxBroadPhaseRegion const* region, bool populateRegion);

    /// Removes a new broad-phase region.
    ///
    /// If the region still contains objects, and if those objects do not overlap any region any more, they are not
    /// automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
    /// is used for each object. Users are responsible for removing the objects from the simulation if this is the
    /// desired behavior.
    ///
    /// If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.
    ///
    /// True if success
bool      physx_PxScene_removeBroadPhaseRegion_mut(physx_PxScene* self_, uint32_t handle);

    /// Get the task manager associated with this scene
    ///
    /// the task manager associated with the scene
physx_PxTaskManager*      physx_PxScene_getTaskManager(physx_PxScene const* self_);

    /// Lock the scene for reading from the calling thread.
    ///
    /// When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockRead() must be
    /// called before any read calls are made on the scene.
    ///
    /// Multiple threads may read at the same time, no threads may read while a thread is writing.
    /// If a call to lockRead() is made while another thread is holding a write lock
    /// then the calling thread will be blocked until the writing thread calls unlockWrite().
    ///
    /// Lock upgrading is *not* supported, that means it is an error to
    /// call lockRead() followed by lockWrite().
    ///
    /// Recursive locking is supported but each lockRead() call must be paired with an unlockRead().
void      physx_PxScene_lockRead_mut(physx_PxScene* self_, char const* file, uint32_t line);

    /// Unlock the scene from reading.
    ///
    /// Each unlockRead() must be paired with a lockRead() from the same thread.
void      physx_PxScene_unlockRead_mut(physx_PxScene* self_);

    /// Lock the scene for writing from this thread.
    ///
    /// When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockWrite() must be
    /// called before any write calls are made on the scene.
    ///
    /// Only one thread may write at a time and no threads may read while a thread is writing.
    /// If a call to lockWrite() is made and there are other threads reading then the
    /// calling thread will be blocked until the readers complete.
    ///
    /// Writers have priority. If a thread is blocked waiting to write then subsequent calls to
    /// lockRead() from other threads will be blocked until the writer completes.
    ///
    /// If multiple threads are waiting to write then the thread that is first
    /// granted access depends on OS scheduling.
    ///
    /// Recursive locking is supported but each lockWrite() call must be paired
    /// with an unlockWrite().
    ///
    /// If a thread has already locked the scene for writing then it may call
    /// lockRead().
void      physx_PxScene_lockWrite_mut(physx_PxScene* self_, char const* file, uint32_t line);

    /// Unlock the scene from writing.
    ///
    /// Each unlockWrite() must be paired with a lockWrite() from the same thread.
void      physx_PxScene_unlockWrite_mut(physx_PxScene* self_);

    /// set the cache blocks that can be used during simulate().
    ///
    /// Each frame the simulation requires memory to store contact, friction, and contact cache data. This memory is used in blocks of 16K.
    /// Each frame the blocks used by the previous frame are freed, and may be retrieved by the application using PxScene::flushSimulation()
    ///
    /// This call will force allocation of cache blocks if the numBlocks parameter is greater than the currently allocated number
    /// of blocks, and less than the max16KContactDataBlocks parameter specified at scene creation time.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setNbContactDataBlocks_mut(physx_PxScene* self_, uint32_t numBlocks);

    /// get the number of cache blocks currently used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the number of cache blocks currently used by the scene
uint32_t      physx_PxScene_getNbContactDataBlocksUsed(physx_PxScene const* self_);

    /// get the maximum number of cache blocks used by the scene
    ///
    /// This function may not be called while the scene is simulating
    ///
    /// the maximum number of cache blocks everused by the scene
uint32_t      physx_PxScene_getMaxNbContactDataBlocksUsed(physx_PxScene const* self_);

    /// Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene
uint32_t      physx_PxScene_getContactReportStreamBufferSize(physx_PxScene const* self_);

    /// Sets the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setSolverBatchSize_mut(physx_PxScene* self_, uint32_t solverBatchSize);

    /// Retrieves the number of actors required to spawn a separate rigid body solver thread.
    ///
    /// Current number of actors required to spawn a separate rigid body solver thread.
uint32_t      physx_PxScene_getSolverBatchSize(physx_PxScene const* self_);

    /// Sets the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Do not use this method while the simulation is running.
void      physx_PxScene_setSolverArticulationBatchSize_mut(physx_PxScene* self_, uint32_t solverBatchSize);

    /// Retrieves the number of articulations required to spawn a separate rigid body solver thread.
    ///
    /// Current number of articulations required to spawn a separate rigid body solver thread.
uint32_t      physx_PxScene_getSolverArticulationBatchSize(physx_PxScene const* self_);

    /// Returns the wake counter reset value.
    ///
    /// Wake counter reset value
float      physx_PxScene_getWakeCounterResetValue(physx_PxScene const* self_);

    /// Shift the scene origin by the specified vector.
    ///
    /// The poses of all objects in the scene and the corresponding data structures will get adjusted to reflect the new origin location
    /// (the shift vector will get subtracted from all object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysX accordingly.
    ///
    /// Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.
    ///
    /// Make sure to propagate the origin shift to other dependent modules (for example, the character controller module etc.).
    ///
    /// This is an expensive operation and we recommend to use it only in the case where distance related precision issues may arise in areas far from the origin.
void      physx_PxScene_shiftOrigin_mut(physx_PxScene* self_, physx_PxVec3 const* shift);

    /// Returns the Pvd client associated with the scene.
    ///
    /// the client, NULL if no PVD supported.
physx_PxPvdSceneClient*      physx_PxScene_getScenePvdClient_mut(physx_PxScene* self_);

    /// Copy GPU articulation data from the internal GPU buffer to a user-provided device buffer.
void      physx_PxScene_copyArticulationData_mut(physx_PxScene* self_, void* data, void* index, int32_t dataType, uint32_t nbCopyArticulations, void* copyEvent);

    /// Apply GPU articulation data from a user-provided device buffer to the internal GPU buffer.
void      physx_PxScene_applyArticulationData_mut(physx_PxScene* self_, void* data, void* index, int32_t dataType, uint32_t nbUpdatedArticulations, void* waitEvent, void* signalEvent);

    /// Copy GPU softbody data from the internal GPU buffer to a user-provided device buffer.
void      physx_PxScene_copySoftBodyData_mut(physx_PxScene* self_, void** data, void* dataSizes, void* softBodyIndices, int32_t flag, uint32_t nbCopySoftBodies, uint32_t maxSize, void* copyEvent);

    /// Apply user-provided data to the internal softbody system.
void      physx_PxScene_applySoftBodyData_mut(physx_PxScene* self_, void** data, void* dataSizes, void* softBodyIndices, int32_t flag, uint32_t nbUpdatedSoftBodies, uint32_t maxSize, void* applyEvent);

    /// Copy contact data from the internal GPU buffer to a user-provided device buffer.
    ///
    /// The contact data contains pointers to internal state and is only valid until the next call to simulate().
void      physx_PxScene_copyContactData_mut(physx_PxScene* self_, void* data, uint32_t maxContactPairs, void* numContactPairs, void* copyEvent);

    /// Copy GPU rigid body data from the internal GPU buffer to a user-provided device buffer.
void      physx_PxScene_copyBodyData_mut(physx_PxScene* self_, physx_PxGpuBodyData* data, physx_PxGpuActorPair* index, uint32_t nbCopyActors, void* copyEvent);

    /// Apply user-provided data to rigid body.
void      physx_PxScene_applyActorData_mut(physx_PxScene* self_, void* data, physx_PxGpuActorPair* index, int32_t flag, uint32_t nbUpdatedActors, void* waitEvent, void* signalEvent);

    /// Compute dense Jacobian matrices for specified articulations on the GPU.
    ///
    /// The size of Jacobians can vary by articulation, since it depends on the number of links, degrees-of-freedom, and whether the base is fixed.
    ///
    /// The size is determined using these formulas:
    /// nCols = (fixedBase ? 0 : 6) + dofCount
    /// nRows = (fixedBase ? 0 : 6) + (linkCount - 1) * 6;
    ///
    /// The user must ensure that adequate space is provided for each Jacobian matrix.
void      physx_PxScene_computeDenseJacobians_mut(physx_PxScene* self_, physx_PxIndexDataPair const* indices, uint32_t nbIndices, void* computeEvent);

    /// Compute the joint-space inertia matrices that maps joint accelerations to joint forces: forces = M * accelerations on the GPU.
    ///
    /// The size of matrices can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount * dofCount
    ///
    /// The user must ensure that adequate space is provided for each mass matrix.
void      physx_PxScene_computeGeneralizedMassMatrices_mut(physx_PxScene* self_, physx_PxIndexDataPair const* indices, uint32_t nbIndices, void* computeEvent);

    /// Computes the joint DOF forces required to counteract gravitational forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
void      physx_PxScene_computeGeneralizedGravityForces_mut(physx_PxScene* self_, physx_PxIndexDataPair const* indices, uint32_t nbIndices, void* computeEvent);

    /// Computes the joint DOF forces required to counteract coriolis and centrifugal forces for the given articulation pose.
    ///
    /// The size of the result can vary by articulation, since it depends on the number of links and degrees-of-freedom.
    ///
    /// The size is determined using this formula:
    /// sizeof(float) * dofCount
    ///
    /// The user must ensure that adequate space is provided for each articulation.
void      physx_PxScene_computeCoriolisAndCentrifugalForces_mut(physx_PxScene* self_, physx_PxIndexDataPair const* indices, uint32_t nbIndices, void* computeEvent);

physx_PxgDynamicsMemoryConfig      physx_PxScene_getGpuDynamicsConfig(physx_PxScene const* self_);

    /// Apply user-provided data to particle buffers.
    ///
    /// This function should be used if the particle buffer flags are already on the device. Otherwise, use PxParticleBuffer::raiseFlags()
    /// from the CPU.
    ///
    /// This assumes the data has been changed directly in the PxParticleBuffer.
void      physx_PxScene_applyParticleBufferData_mut(physx_PxScene* self_, uint32_t const* indices, physx_PxGpuParticleBufferIndexPair const* bufferIndexPair, uint32_t const* flags, uint32_t nbUpdatedBuffers, void* waitEvent, void* signalEvent);

    /// Constructor
physx_PxSceneReadLock*      physx_PxSceneReadLock_new_alloc(physx_PxScene* scene, char const* file, uint32_t line);

void      physx_PxSceneReadLock_delete(physx_PxSceneReadLock* self_);

    /// Constructor
physx_PxSceneWriteLock*      physx_PxSceneWriteLock_new_alloc(physx_PxScene* scene, char const* file, uint32_t line);

void      physx_PxSceneWriteLock_delete(physx_PxSceneWriteLock* self_);

physx_PxContactPairExtraDataItem      physx_PxContactPairExtraDataItem_new();

physx_PxContactPairVelocity      physx_PxContactPairVelocity_new();

physx_PxContactPairPose      physx_PxContactPairPose_new();

physx_PxContactPairIndex      physx_PxContactPairIndex_new();

    /// Constructor
physx_PxContactPairExtraDataIterator      physx_PxContactPairExtraDataIterator_new(uint8_t const* stream, uint32_t size);

    /// Advances the iterator to next set of extra data items.
    ///
    /// The contact pair extra data stream contains sets of items as requested by the corresponding [`PxPairFlag`] flags
    /// [`PxPairFlag::ePRE_SOLVER_VELOCITY`], #PxPairFlag::ePOST_SOLVER_VELOCITY, #PxPairFlag::eCONTACT_EVENT_POSE. A set can contain one
    /// item of each plus the PxContactPairIndex item. This method parses the stream and points the iterator
    /// member variables to the corresponding items of the current set, if they are available. If CCD is not enabled,
    /// you should only get one set of items. If CCD with multiple passes is enabled, you might get more than one item
    /// set.
    ///
    /// Even though contact pair extra data is requested per shape pair, you will not get an item set per shape pair
    /// but one per actor pair. If, for example, an actor has two shapes and both collide with another actor, then
    /// there will only be one item set (since it applies to both shape pairs).
    ///
    /// True if there was another set of extra data items in the stream, else false.
bool      physx_PxContactPairExtraDataIterator_nextItemSet_mut(physx_PxContactPairExtraDataIterator* self_);

physx_PxContactPairHeader      physx_PxContactPairHeader_new();

physx_PxContactPair      physx_PxContactPair_new();

    /// Extracts the contact points from the stream and stores them in a convenient format.
    ///
    /// Number of contact points written to the buffer.
uint32_t      physx_PxContactPair_extractContacts(physx_PxContactPair const* self_, physx_PxContactPairPoint* userBuffer, uint32_t bufferSize);

    /// Helper method to clone the contact pair and copy the contact data stream into a user buffer.
    ///
    /// The contact data stream is only accessible during the contact report callback. This helper function provides copy functionality
    /// to buffer the contact stream information such that it can get accessed at a later stage.
void      physx_PxContactPair_bufferContacts(physx_PxContactPair const* self_, physx_PxContactPair* newPair, uint8_t* bufferMemory);

uint32_t const*      physx_PxContactPair_getInternalFaceIndices(physx_PxContactPair const* self_);

physx_PxTriggerPair      physx_PxTriggerPair_new();

physx_PxConstraintInfo      physx_PxConstraintInfo_new();

physx_PxConstraintInfo      physx_PxConstraintInfo_new_1(physx_PxConstraint* c, void* extRef, uint32_t t);

    /// This is called when a breakable constraint breaks.
    ///
    /// The user should not release the constraint shader inside this call!
    ///
    /// No event will get reported if the constraint breaks but gets deleted while the time step is still being simulated.
void      physx_PxSimulationEventCallback_onConstraintBreak_mut(physx_PxSimulationEventCallback* self_, physx_PxConstraintInfo* constraints, uint32_t count);

    /// This is called with the actors which have just been woken up.
    ///
    /// Only supported by rigid bodies yet.
    ///
    /// Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
    ///
    /// Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
    /// will get reported. For example, let us assume actor A is awake, then A->putToSleep() gets called, then later A->wakeUp() gets called.
    /// At the next simulate/fetchResults() step only an onWake() event will get triggered because that was the last transition.
    ///
    /// If an actor gets newly added to a scene with properties such that it is awake and the sleep state does not get changed by
    /// the user or simulation, then an onWake() event will get sent at the next simulate/fetchResults() step.
void      physx_PxSimulationEventCallback_onWake_mut(physx_PxSimulationEventCallback* self_, physx_PxActor** actors, uint32_t count);

    /// This is called with the actors which have just been put to sleep.
    ///
    /// Only supported by rigid bodies yet.
    ///
    /// Only called on actors for which the PxActorFlag eSEND_SLEEP_NOTIFIES has been set.
    ///
    /// Only the latest sleep state transition happening between fetchResults() of the previous frame and fetchResults() of the current frame
    /// will get reported. For example, let us assume actor A is asleep, then A->wakeUp() gets called, then later A->putToSleep() gets called.
    /// At the next simulate/fetchResults() step only an onSleep() event will get triggered because that was the last transition (assuming the simulation
    /// does not wake the actor up).
    ///
    /// If an actor gets newly added to a scene with properties such that it is asleep and the sleep state does not get changed by
    /// the user or simulation, then an onSleep() event will get sent at the next simulate/fetchResults() step.
void      physx_PxSimulationEventCallback_onSleep_mut(physx_PxSimulationEventCallback* self_, physx_PxActor** actors, uint32_t count);

    /// This is called when certain contact events occur.
    ///
    /// The method will be called for a pair of actors if one of the colliding shape pairs requested contact notification.
    /// You request which events are reported using the filter shader/callback mechanism (see [`PxSimulationFilterShader`],
    /// [`PxSimulationFilterCallback`], #PxPairFlag).
    ///
    /// Do not keep references to the passed objects, as they will be
    /// invalid after this function returns.
void      physx_PxSimulationEventCallback_onContact_mut(physx_PxSimulationEventCallback* self_, physx_PxContactPairHeader const* pairHeader, physx_PxContactPair const* pairs, uint32_t nbPairs);

    /// This is called with the current trigger pair events.
    ///
    /// Shapes which have been marked as triggers using PxShapeFlag::eTRIGGER_SHAPE will send events
    /// according to the pair flag specification in the filter shader (see [`PxPairFlag`], #PxSimulationFilterShader).
    ///
    /// Trigger shapes will no longer send notification events for interactions with other trigger shapes.
void      physx_PxSimulationEventCallback_onTrigger_mut(physx_PxSimulationEventCallback* self_, physx_PxTriggerPair* pairs, uint32_t count);

    /// Provides early access to the new pose of moving rigid bodies.
    ///
    /// When this call occurs, rigid bodies having the [`PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW`]
    /// flag set, were moved by the simulation and their new poses can be accessed through the provided buffers.
    ///
    /// The provided buffers are valid and can be read until the next call to [`PxScene::simulate`]() or #PxScene::collide().
    ///
    /// This callback gets triggered while the simulation is running. If the provided rigid body references are used to
    /// read properties of the object, then the callback has to guarantee no other thread is writing to the same body at the same
    /// time.
    ///
    /// The code in this callback should be lightweight as it can block the simulation, that is, the
    /// [`PxScene::fetchResults`]() call.
void      physx_PxSimulationEventCallback_onAdvance_mut(physx_PxSimulationEventCallback* self_, physx_PxRigidBody const* const* bodyBuffer, physx_PxTransform const* poseBuffer, uint32_t count);

void      physx_PxSimulationEventCallback_delete(physx_PxSimulationEventCallback* self_);

physx_PxFEMParameters      physx_PxFEMParameters_new();

    /// Release this object.
void      physx_PxPruningStructure_release_mut(physx_PxPruningStructure* self_);

    /// Retrieve rigid actors in the pruning structure.
    ///
    /// You can retrieve the number of rigid actor pointers by calling [`getNbRigidActors`]()
    ///
    /// Number of rigid actor pointers written to the buffer.
uint32_t      physx_PxPruningStructure_getRigidActors(physx_PxPruningStructure const* self_, physx_PxRigidActor** userBuffer, uint32_t bufferSize, uint32_t startIndex);

    /// Returns the number of rigid actors in the pruning structure.
    ///
    /// You can use [`getRigidActors`]() to retrieve the rigid actor pointers.
    ///
    /// Number of rigid actors in the pruning structure.
uint32_t      physx_PxPruningStructure_getNbRigidActors(physx_PxPruningStructure const* self_);

    /// Gets the merge data for static actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for static actors.
void const*      physx_PxPruningStructure_getStaticMergeData(physx_PxPruningStructure const* self_);

    /// Gets the merge data for dynamic actors
    ///
    /// This is mainly called by the PxSceneQuerySystem::merge() function to merge a PxPruningStructure
    /// with the internal data-structures of the scene-query system.
    ///
    /// Implementation-dependent merge data for dynamic actors.
void const*      physx_PxPruningStructure_getDynamicMergeData(physx_PxPruningStructure const* self_);

char const*      physx_PxPruningStructure_getConcreteTypeName(physx_PxPruningStructure const* self_);

physx_PxExtendedVec3      physx_PxExtendedVec3_new();

physx_PxExtendedVec3      physx_PxExtendedVec3_new_1(double _x, double _y, double _z);

bool      physx_PxExtendedVec3_isZero(physx_PxExtendedVec3 const* self_);

double      physx_PxExtendedVec3_dot(physx_PxExtendedVec3 const* self_, physx_PxVec3 const* v);

double      physx_PxExtendedVec3_distanceSquared(physx_PxExtendedVec3 const* self_, physx_PxExtendedVec3 const* v);

double      physx_PxExtendedVec3_magnitudeSquared(physx_PxExtendedVec3 const* self_);

double      physx_PxExtendedVec3_magnitude(physx_PxExtendedVec3 const* self_);

double      physx_PxExtendedVec3_normalize_mut(physx_PxExtendedVec3* self_);

bool      physx_PxExtendedVec3_isFinite(physx_PxExtendedVec3 const* self_);

void      physx_PxExtendedVec3_maximum_mut(physx_PxExtendedVec3* self_, physx_PxExtendedVec3 const* v);

void      physx_PxExtendedVec3_minimum_mut(physx_PxExtendedVec3* self_, physx_PxExtendedVec3 const* v);

void      physx_PxExtendedVec3_set_mut(physx_PxExtendedVec3* self_, double x_, double y_, double z_);

void      physx_PxExtendedVec3_setPlusInfinity_mut(physx_PxExtendedVec3* self_);

void      physx_PxExtendedVec3_setMinusInfinity_mut(physx_PxExtendedVec3* self_);

void      physx_PxExtendedVec3_cross_mut(physx_PxExtendedVec3* self_, physx_PxExtendedVec3 const* left, physx_PxVec3 const* right);

void      physx_PxExtendedVec3_cross_mut_1(physx_PxExtendedVec3* self_, physx_PxExtendedVec3 const* left, physx_PxExtendedVec3 const* right);

physx_PxExtendedVec3      physx_PxExtendedVec3_cross(physx_PxExtendedVec3 const* self_, physx_PxExtendedVec3 const* v);

void      physx_PxExtendedVec3_cross_mut_2(physx_PxExtendedVec3* self_, physx_PxVec3 const* left, physx_PxExtendedVec3 const* right);

physx_PxVec3      physx_phys_toVec3(physx_PxExtendedVec3 const* v);

int32_t      physx_PxObstacle_getType(physx_PxObstacle const* self_);

physx_PxBoxObstacle      physx_PxBoxObstacle_new();

physx_PxCapsuleObstacle      physx_PxCapsuleObstacle_new();

    /// Releases the context.
void      physx_PxObstacleContext_release_mut(physx_PxObstacleContext* self_);

    /// Retrieves the controller manager associated with this context.
    ///
    /// The associated controller manager
physx_PxControllerManager*      physx_PxObstacleContext_getControllerManager(physx_PxObstacleContext const* self_);

    /// Adds an obstacle to the context.
    ///
    /// Handle for newly-added obstacle
uint32_t      physx_PxObstacleContext_addObstacle_mut(physx_PxObstacleContext* self_, physx_PxObstacle const* obstacle);

    /// Removes an obstacle from the context.
    ///
    /// True if success
bool      physx_PxObstacleContext_removeObstacle_mut(physx_PxObstacleContext* self_, uint32_t handle);

    /// Updates data for an existing obstacle.
    ///
    /// True if success
bool      physx_PxObstacleContext_updateObstacle_mut(physx_PxObstacleContext* self_, uint32_t handle, physx_PxObstacle const* obstacle);

    /// Retrieves number of obstacles in the context.
    ///
    /// Number of obstacles in the context
uint32_t      physx_PxObstacleContext_getNbObstacles(physx_PxObstacleContext const* self_);

    /// Retrieves desired obstacle.
    ///
    /// Desired obstacle
physx_PxObstacle const*      physx_PxObstacleContext_getObstacle(physx_PxObstacleContext const* self_, uint32_t i);

    /// Retrieves desired obstacle by given handle.
    ///
    /// Desired obstacle
physx_PxObstacle const*      physx_PxObstacleContext_getObstacleByHandle(physx_PxObstacleContext const* self_, uint32_t handle);

    /// Called when current controller hits a shape.
    ///
    /// This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.
void      physx_PxUserControllerHitReport_onShapeHit_mut(physx_PxUserControllerHitReport* self_, physx_PxControllerShapeHit const* hit);

    /// Called when current controller hits another controller.
void      physx_PxUserControllerHitReport_onControllerHit_mut(physx_PxUserControllerHitReport* self_, physx_PxControllersHit const* hit);

    /// Called when current controller hits a user-defined obstacle.
void      physx_PxUserControllerHitReport_onObstacleHit_mut(physx_PxUserControllerHitReport* self_, physx_PxControllerObstacleHit const* hit);

void      physx_PxControllerFilterCallback_delete(physx_PxControllerFilterCallback* self_);

    /// Filtering method for CCT-vs-CCT.
    ///
    /// true to keep the pair, false to filter it out
bool      physx_PxControllerFilterCallback_filter_mut(physx_PxControllerFilterCallback* self_, physx_PxController const* a, physx_PxController const* b);

physx_PxControllerFilters      physx_PxControllerFilters_new(physx_PxFilterData const* filterData, physx_PxQueryFilterCallback* cb, physx_PxControllerFilterCallback* cctFilterCb);

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
bool      physx_PxControllerDesc_isValid(physx_PxControllerDesc const* self_);

    /// Returns the character controller type
    ///
    /// The controllers type.
int32_t      physx_PxControllerDesc_getType(physx_PxControllerDesc const* self_);

    /// Return the type of controller
int32_t      physx_PxController_getType(physx_PxController const* self_);

    /// Releases the controller.
void      physx_PxController_release_mut(physx_PxController* self_);

    /// Moves the character using a "collide-and-slide" algorithm.
    ///
    /// Collision flags, collection of ::PxControllerCollisionFlags
uint8_t      physx_PxController_move_mut(physx_PxController* self_, physx_PxVec3 const* disp, float minDist, float elapsedTime, physx_PxControllerFilters const* filters, physx_PxObstacleContext const* obstacles);

    /// Sets controller's position.
    ///
    /// The position controlled by this function is the center of the collision shape.
    ///
    /// This is a 'teleport' function, it doesn't check for collisions.
    ///
    /// The character's position must be such that it does not overlap the static geometry.
    ///
    /// To move the character under normal conditions use the [`move`]() function.
    ///
    /// Currently always returns true.
bool      physx_PxController_setPosition_mut(physx_PxController* self_, physx_PxExtendedVec3 const* position);

    /// Retrieve the raw position of the controller.
    ///
    /// The position retrieved by this function is the center of the collision shape. To retrieve the bottom position of the shape,
    /// a.k.a. the foot position, use the getFootPosition() function.
    ///
    /// The position is updated by calls to move(). Calling this method without calling
    /// move() will return the last position or the initial position of the controller.
    ///
    /// The controller's center position
physx_PxExtendedVec3 const*      physx_PxController_getPosition(physx_PxController const* self_);

    /// Set controller's foot position.
    ///
    /// The position controlled by this function is the bottom of the collision shape, a.k.a. the foot position.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// This is a 'teleport' function, it doesn't check for collisions.
    ///
    /// To move the character under normal conditions use the [`move`]() function.
    ///
    /// Currently always returns true.
bool      physx_PxController_setFootPosition_mut(physx_PxController* self_, physx_PxExtendedVec3 const* position);

    /// Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.
    ///
    /// The foot position takes the contact offset into account
    ///
    /// The controller's foot position
physx_PxExtendedVec3      physx_PxController_getFootPosition(physx_PxController const* self_);

    /// Get the rigid body actor associated with this controller (see PhysX documentation).
    /// The behavior upon manually altering this actor is undefined, you should primarily
    /// use it for reading const properties.
    ///
    /// the actor associated with the controller.
physx_PxRigidDynamic*      physx_PxController_getActor(physx_PxController const* self_);

    /// The step height.
void      physx_PxController_setStepOffset_mut(physx_PxController* self_, float offset);

    /// Retrieve the step height.
    ///
    /// The step offset for the controller.
float      physx_PxController_getStepOffset(physx_PxController const* self_);

    /// Sets the non-walkable mode for the CCT.
void      physx_PxController_setNonWalkableMode_mut(physx_PxController* self_, int32_t flag);

    /// Retrieves the non-walkable mode for the CCT.
    ///
    /// The current non-walkable mode.
int32_t      physx_PxController_getNonWalkableMode(physx_PxController const* self_);

    /// Retrieve the contact offset.
    ///
    /// The contact offset for the controller.
float      physx_PxController_getContactOffset(physx_PxController const* self_);

    /// Sets the contact offset.
void      physx_PxController_setContactOffset_mut(physx_PxController* self_, float offset);

    /// Retrieve the 'up' direction.
    ///
    /// The up direction for the controller.
physx_PxVec3      physx_PxController_getUpDirection(physx_PxController const* self_);

    /// Sets the 'up' direction.
void      physx_PxController_setUpDirection_mut(physx_PxController* self_, physx_PxVec3 const* up);

    /// Retrieve the slope limit.
    ///
    /// The slope limit for the controller.
float      physx_PxController_getSlopeLimit(physx_PxController const* self_);

    /// Sets the slope limit.
    ///
    /// This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
    /// (which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
    /// will be ignored.
void      physx_PxController_setSlopeLimit_mut(physx_PxController* self_, float slopeLimit);

    /// Flushes internal geometry cache.
    ///
    /// The character controller uses caching in order to speed up collision testing. The cache is
    /// automatically flushed when a change to static objects is detected in the scene. For example when a
    /// static shape is added, updated, or removed from the scene, the cache is automatically invalidated.
    ///
    /// However there may be situations that cannot be automatically detected, and those require manual
    /// invalidation of the cache. Currently the user must call this when the filtering behavior changes (the
    /// PxControllerFilters parameter of the PxController::move call).  While the controller in principle
    /// could detect a change in these parameters, it cannot detect a change in the behavior of the filtering
    /// function.
void      physx_PxController_invalidateCache_mut(physx_PxController* self_);

    /// Retrieve the scene associated with the controller.
    ///
    /// The physics scene
physx_PxScene*      physx_PxController_getScene_mut(physx_PxController* self_);

    /// Returns the user data associated with this controller.
    ///
    /// The user pointer associated with the controller.
void*      physx_PxController_getUserData(physx_PxController const* self_);

    /// Sets the user data associated with this controller.
void      physx_PxController_setUserData_mut(physx_PxController* self_, void* userData);

    /// Returns information about the controller's internal state.
void      physx_PxController_getState(physx_PxController const* self_, physx_PxControllerState* state);

    /// Returns the controller's internal statistics.
void      physx_PxController_getStats(physx_PxController const* self_, physx_PxControllerStats* stats);

    /// Resizes the controller.
    ///
    /// This function attempts to resize the controller to a given size, while making sure the bottom
    /// position of the controller remains constant. In other words the function modifies both the
    /// height and the (center) position of the controller. This is a helper function that can be used
    /// to implement a 'crouch' functionality for example.
void      physx_PxController_resize_mut(physx_PxController* self_, float height);

    /// constructor sets to default.
physx_PxBoxControllerDesc*      physx_PxBoxControllerDesc_new_alloc();

void      physx_PxBoxControllerDesc_delete(physx_PxBoxControllerDesc* self_);

    /// (re)sets the structure to the default.
void      physx_PxBoxControllerDesc_setToDefault_mut(physx_PxBoxControllerDesc* self_);

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
bool      physx_PxBoxControllerDesc_isValid(physx_PxBoxControllerDesc const* self_);

    /// Gets controller's half height.
    ///
    /// The half height of the controller.
float      physx_PxBoxController_getHalfHeight(physx_PxBoxController const* self_);

    /// Gets controller's half side extent.
    ///
    /// The half side extent of the controller.
float      physx_PxBoxController_getHalfSideExtent(physx_PxBoxController const* self_);

    /// Gets controller's half forward extent.
    ///
    /// The half forward extent of the controller.
float      physx_PxBoxController_getHalfForwardExtent(physx_PxBoxController const* self_);

    /// Sets controller's half height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
bool      physx_PxBoxController_setHalfHeight_mut(physx_PxBoxController* self_, float halfHeight);

    /// Sets controller's half side extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
bool      physx_PxBoxController_setHalfSideExtent_mut(physx_PxBoxController* self_, float halfSideExtent);

    /// Sets controller's half forward extent.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
bool      physx_PxBoxController_setHalfForwardExtent_mut(physx_PxBoxController* self_, float halfForwardExtent);

    /// constructor sets to default.
physx_PxCapsuleControllerDesc*      physx_PxCapsuleControllerDesc_new_alloc();

void      physx_PxCapsuleControllerDesc_delete(physx_PxCapsuleControllerDesc* self_);

    /// (re)sets the structure to the default.
void      physx_PxCapsuleControllerDesc_setToDefault_mut(physx_PxCapsuleControllerDesc* self_);

    /// returns true if the current settings are valid
    ///
    /// True if the descriptor is valid.
bool      physx_PxCapsuleControllerDesc_isValid(physx_PxCapsuleControllerDesc const* self_);

    /// Gets controller's radius.
    ///
    /// The radius of the controller.
float      physx_PxCapsuleController_getRadius(physx_PxCapsuleController const* self_);

    /// Sets controller's radius.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
bool      physx_PxCapsuleController_setRadius_mut(physx_PxCapsuleController* self_, float radius);

    /// Gets controller's height.
    ///
    /// The height of the capsule controller.
float      physx_PxCapsuleController_getHeight(physx_PxCapsuleController const* self_);

    /// Resets controller's height.
    ///
    /// this doesn't check for collisions.
    ///
    /// Currently always true.
bool      physx_PxCapsuleController_setHeight_mut(physx_PxCapsuleController* self_, float height);

    /// Gets controller's climbing mode.
    ///
    /// The capsule controller's climbing mode.
int32_t      physx_PxCapsuleController_getClimbingMode(physx_PxCapsuleController const* self_);

    /// Sets controller's climbing mode.
bool      physx_PxCapsuleController_setClimbingMode_mut(physx_PxCapsuleController* self_, int32_t mode);

    /// Retrieve behavior flags for a shape.
    ///
    /// When the CCT touches a shape, the CCT's behavior w.r.t. this shape can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given shape
uint8_t      physx_PxControllerBehaviorCallback_getBehaviorFlags_mut(physx_PxControllerBehaviorCallback* self_, physx_PxShape const* shape, physx_PxActor const* actor);

    /// Retrieve behavior flags for a controller.
    ///
    /// When the CCT touches a controller, the CCT's behavior w.r.t. this controller can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// The flag PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT is not supported.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given controller
uint8_t      physx_PxControllerBehaviorCallback_getBehaviorFlags_mut_1(physx_PxControllerBehaviorCallback* self_, physx_PxController const* controller);

    /// Retrieve behavior flags for an obstacle.
    ///
    /// When the CCT touches an obstacle, the CCT's behavior w.r.t. this obstacle can be customized by users.
    /// This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.
    ///
    /// See comments about deprecated functions at the start of this class
    ///
    /// Desired behavior flags for the given obstacle
uint8_t      physx_PxControllerBehaviorCallback_getBehaviorFlags_mut_2(physx_PxControllerBehaviorCallback* self_, physx_PxObstacle const* obstacle);

    /// Releases the controller manager.
    ///
    /// This will release all associated controllers and obstacle contexts.
    ///
    /// This function is required to be called to release foundation usage.
void      physx_PxControllerManager_release_mut(physx_PxControllerManager* self_);

    /// Returns the scene the manager is adding the controllers to.
    ///
    /// The associated physics scene.
physx_PxScene*      physx_PxControllerManager_getScene(physx_PxControllerManager const* self_);

    /// Returns the number of controllers that are being managed.
    ///
    /// The number of controllers.
uint32_t      physx_PxControllerManager_getNbControllers(physx_PxControllerManager const* self_);

    /// Retrieve one of the controllers in the manager.
    ///
    /// The controller with the specified index.
physx_PxController*      physx_PxControllerManager_getController_mut(physx_PxControllerManager* self_, uint32_t index);

    /// Creates a new character controller.
    ///
    /// The new controller
physx_PxController*      physx_PxControllerManager_createController_mut(physx_PxControllerManager* self_, physx_PxControllerDesc const* desc);

    /// Releases all the controllers that are being managed.
void      physx_PxControllerManager_purgeControllers_mut(physx_PxControllerManager* self_);

    /// Retrieves debug data.
    ///
    /// The render buffer filled with debug-render data
physx_PxRenderBuffer*      physx_PxControllerManager_getRenderBuffer_mut(physx_PxControllerManager* self_);

    /// Sets debug rendering flags
void      physx_PxControllerManager_setDebugRenderingFlags_mut(physx_PxControllerManager* self_, uint32_t flags);

    /// Returns the number of obstacle contexts that are being managed.
    ///
    /// The number of obstacle contexts.
uint32_t      physx_PxControllerManager_getNbObstacleContexts(physx_PxControllerManager const* self_);

    /// Retrieve one of the obstacle contexts in the manager.
    ///
    /// The obstacle context with the specified index.
physx_PxObstacleContext*      physx_PxControllerManager_getObstacleContext_mut(physx_PxControllerManager* self_, uint32_t index);

    /// Creates an obstacle context.
    ///
    /// New obstacle context
physx_PxObstacleContext*      physx_PxControllerManager_createObstacleContext_mut(physx_PxControllerManager* self_);

    /// Computes character-character interactions.
    ///
    /// This function is an optional helper to properly resolve interactions between characters, in case they overlap (which can happen for gameplay reasons, etc).
    ///
    /// You should call this once per frame, before your PxController::move() calls. The function will not move the characters directly, but it will
    /// compute overlap information for each character that will be used in the next move() call.
    ///
    /// You need to provide a proper time value here so that interactions are resolved in a way that do not depend on the framerate.
    ///
    /// If you only have one character in the scene, or if you can guarantee your characters will never overlap, then you do not need to call this function.
    ///
    /// Releasing the manager will automatically release all the associated obstacle contexts.
void      physx_PxControllerManager_computeInteractions_mut(physx_PxControllerManager* self_, float elapsedTime, physx_PxControllerFilterCallback* cctFilterCb);

    /// Enables or disables runtime tessellation.
    ///
    /// Large triangles can create accuracy issues in the sweep code, which in turn can lead to characters not sliding smoothly
    /// against geometries, or even penetrating them. This feature allows one to reduce those issues by tessellating large
    /// triangles at runtime, before performing sweeps against them. The amount of tessellation is controlled by the 'maxEdgeLength' parameter.
    /// Any triangle with at least one edge length greater than the maxEdgeLength will get recursively tessellated, until resulting triangles are small enough.
    ///
    /// This features only applies to triangle meshes, convex meshes, heightfields and boxes.
void      physx_PxControllerManager_setTessellation_mut(physx_PxControllerManager* self_, bool flag, float maxEdgeLength);

    /// Enables or disables the overlap recovery module.
    ///
    /// The overlap recovery module can be used to depenetrate CCTs from static objects when an overlap is detected. This can happen
    /// in three main cases:
    /// - when the CCT is directly spawned or teleported in another object
    /// - when the CCT algorithm fails due to limited FPU accuracy
    /// - when the "up vector" is modified, making the rotated CCT shape overlap surrounding objects
    ///
    /// When activated, the CCT module will automatically try to resolve the penetration, and move the CCT to a safe place where it does
    /// not overlap other objects anymore. This only concerns static objects, dynamic objects are ignored by the recovery module.
    ///
    /// When the recovery module is not activated, it is possible for the CCTs to go through static objects. By default, the recovery
    /// module is enabled.
    ///
    /// The recovery module currently works with all geometries except heightfields.
void      physx_PxControllerManager_setOverlapRecoveryModule_mut(physx_PxControllerManager* self_, bool flag);

    /// Enables or disables the precise sweeps.
    ///
    /// Precise sweeps are more accurate, but also potentially slower than regular sweeps.
    ///
    /// By default, precise sweeps are enabled.
void      physx_PxControllerManager_setPreciseSweeps_mut(physx_PxControllerManager* self_, bool flag);

    /// Enables or disables vertical sliding against ceilings.
    ///
    /// Geometry is seen as "ceilings" when the following condition is met:
    ///
    /// dot product(contact normal, up direction)
    /// <
    /// 0.0f
    ///
    /// This flag controls whether characters should slide vertically along the geometry in that case.
    ///
    /// By default, sliding is allowed.
void      physx_PxControllerManager_setPreventVerticalSlidingAgainstCeiling_mut(physx_PxControllerManager* self_, bool flag);

    /// Shift the origin of the character controllers and obstacle objects by the specified vector.
    ///
    /// The positions of all character controllers, obstacle objects and the corresponding data structures will get adjusted to reflect the shifted origin location
    /// (the shift vector will get subtracted from all character controller and obstacle object positions).
    ///
    /// It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysXCharacterKinematic accordingly.
    ///
    /// This call will not automatically shift the PhysX scene and its objects. You need to call PxScene::shiftOrigin() seperately to keep the systems in sync.
void      physx_PxControllerManager_shiftOrigin_mut(physx_PxControllerManager* self_, physx_PxVec3 const* shift);

    /// Creates the controller manager.
    ///
    /// The character controller is informed by [`PxDeletionListener::onRelease`]() when actors or shapes are released, and updates its internal
    /// caches accordingly. If character controller movement or a call to [`PxControllerManager::shiftOrigin`]() may overlap with actor/shape releases,
    /// internal data structures must be guarded against concurrent access.
    ///
    /// Locking guarantees thread safety in such scenarios.
    ///
    /// locking may result in significant slowdown for release of actors or shapes.
    ///
    /// By default, locking is disabled.
physx_PxControllerManager*      physx_phys_PxCreateControllerManager(physx_PxScene* scene, bool lockingEnabled);

physx_PxDim3      physx_PxDim3_new();

    /// Constructor
physx_PxSDFDesc      physx_PxSDFDesc_new();

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
bool      physx_PxSDFDesc_isValid(physx_PxSDFDesc const* self_);

    /// constructor sets to default.
physx_PxConvexMeshDesc      physx_PxConvexMeshDesc_new();

    /// (re)sets the structure to the default.
void      physx_PxConvexMeshDesc_setToDefault_mut(physx_PxConvexMeshDesc* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// True if the current settings are valid
bool      physx_PxConvexMeshDesc_isValid(physx_PxConvexMeshDesc const* self_);

    /// Constructor sets to default.
physx_PxTriangleMeshDesc      physx_PxTriangleMeshDesc_new();

    /// (re)sets the structure to the default.
void      physx_PxTriangleMeshDesc_setToDefault_mut(physx_PxTriangleMeshDesc* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid
bool      physx_PxTriangleMeshDesc_isValid(physx_PxTriangleMeshDesc const* self_);

    /// Constructor to build an empty tetmesh description
physx_PxTetrahedronMeshDesc      physx_PxTetrahedronMeshDesc_new();

bool      physx_PxTetrahedronMeshDesc_isValid(physx_PxTetrahedronMeshDesc const* self_);

    /// Constructor to build an empty simulation description
physx_PxSoftBodySimulationDataDesc      physx_PxSoftBodySimulationDataDesc_new();

bool      physx_PxSoftBodySimulationDataDesc_isValid(physx_PxSoftBodySimulationDataDesc const* self_);

    /// Desc initialization to default value.
void      physx_PxBVH34MidphaseDesc_setToDefault_mut(physx_PxBVH34MidphaseDesc* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
bool      physx_PxBVH34MidphaseDesc_isValid(physx_PxBVH34MidphaseDesc const* self_);

physx_PxMidphaseDesc      physx_PxMidphaseDesc_new();

    /// Returns type of midphase mesh structure.
    ///
    /// PxMeshMidPhase::Enum
int32_t      physx_PxMidphaseDesc_getType(physx_PxMidphaseDesc const* self_);

    /// Initialize the midphase mesh structure descriptor
void      physx_PxMidphaseDesc_setToDefault_mut(physx_PxMidphaseDesc* self_, int32_t type_);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
bool      physx_PxMidphaseDesc_isValid(physx_PxMidphaseDesc const* self_);

physx_PxBVHDesc      physx_PxBVHDesc_new();

    /// Initialize the BVH descriptor
void      physx_PxBVHDesc_setToDefault_mut(physx_PxBVHDesc* self_);

    /// Returns true if the descriptor is valid.
    ///
    /// true if the current settings are valid.
bool      physx_PxBVHDesc_isValid(physx_PxBVHDesc const* self_);

physx_PxCookingParams      physx_PxCookingParams_new(physx_PxTolerancesScale const* sc);

physx_PxInsertionCallback*      physx_phys_PxGetStandaloneInsertionCallback();

    /// Cooks a bounding volume hierarchy. The results are written to the stream.
    ///
    /// PxCookBVH() allows a BVH description to be cooked into a binary stream
    /// suitable for loading and performing BVH detection at runtime.
    ///
    /// true on success.
bool      physx_phys_PxCookBVH(physx_PxBVHDesc const* desc, physx_PxOutputStream* stream);

    /// Cooks and creates a bounding volume hierarchy without going through a stream.
    ///
    /// This method does the same as cookBVH, but the produced BVH is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxBVH pointer on success
physx_PxBVH*      physx_phys_PxCreateBVH(physx_PxBVHDesc const* desc, physx_PxInsertionCallback* insertionCallback);

    /// Cooks a heightfield. The results are written to the stream.
    ///
    /// To create a heightfield object there is an option to precompute some of calculations done while loading the heightfield data.
    ///
    /// cookHeightField() allows a heightfield description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
bool      physx_phys_PxCookHeightField(physx_PxHeightFieldDesc const* desc, physx_PxOutputStream* stream);

    /// Cooks and creates a heightfield mesh and inserts it into PxPhysics.
    ///
    /// PxHeightField pointer on success
physx_PxHeightField*      physx_phys_PxCreateHeightField(physx_PxHeightFieldDesc const* desc, physx_PxInsertionCallback* insertionCallback);

    /// Cooks a convex mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// cookConvexMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// The number of vertices and the number of convex polygons in a cooked convex mesh is limited to 255.
    ///
    /// If those limits are exceeded in either the user-provided data or the final cooked mesh, an error is reported.
    ///
    /// true on success.
bool      physx_phys_PxCookConvexMesh(physx_PxCookingParams const* params, physx_PxConvexMeshDesc const* desc, physx_PxOutputStream* stream, int32_t* condition);

    /// Cooks and creates a convex mesh without going through a stream.
    ///
    /// This method does the same as cookConvexMesh, but the produced mesh is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxConvexMesh pointer on success
physx_PxConvexMesh*      physx_phys_PxCreateConvexMesh(physx_PxCookingParams const* params, physx_PxConvexMeshDesc const* desc, physx_PxInsertionCallback* insertionCallback, int32_t* condition);

    /// Verifies if the convex mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The convex mesh descriptor must contain an already created convex mesh - the vertices, indices and polygons must be provided.
    ///
    /// This function should be used if PxConvexFlag::eDISABLE_MESH_VALIDATION is planned to be used in release builds.
    ///
    /// true if all the validity conditions hold, false otherwise.
bool      physx_phys_PxValidateConvexMesh(physx_PxCookingParams const* params, physx_PxConvexMeshDesc const* desc);

    /// Computed hull polygons from given vertices and triangles. Polygons are needed for PxConvexMeshDesc rather than triangles.
    ///
    /// Please note that the resulting polygons may have different number of vertices. Some vertices may be removed.
    /// The output vertices, indices and polygons must be used to construct a hull.
    ///
    /// The provided PxAllocatorCallback does allocate the out array's. It is the user responsibility to deallocated those
    /// array's.
    ///
    /// true on success
bool      physx_phys_PxComputeHullPolygons(physx_PxCookingParams const* params, physx_PxSimpleTriangleMesh const* mesh, physx_PxAllocatorCallback* inCallback, uint32_t* nbVerts, physx_PxVec3** vertices, uint32_t* nbIndices, uint32_t** indices, uint32_t* nbPolygons, physx_PxHullPolygon** hullPolygons);

    /// Verifies if the triangle mesh is valid. Prints an error message for each inconsistency found.
    ///
    /// The following conditions are true for a valid triangle mesh:
    /// 1. There are no duplicate vertices (within specified vertexWeldTolerance. See PxCookingParams::meshWeldTolerance)
    /// 2. There are no large triangles (within specified PxTolerancesScale.)
    ///
    /// true if all the validity conditions hold, false otherwise.
bool      physx_phys_PxValidateTriangleMesh(physx_PxCookingParams const* params, physx_PxTriangleMeshDesc const* desc);

    /// Cooks and creates a triangle mesh without going through a stream.
    ///
    /// This method does the same as cookTriangleMesh, but the produced mesh is not stored
    /// into a stream but is either directly inserted in PxPhysics, or created as a standalone
    /// object. Use this method if you are unable to cook offline.
    ///
    /// PxInsertionCallback can be obtained through PxPhysics::getPhysicsInsertionCallback()
    /// or PxCooking::getStandaloneInsertionCallback().
    ///
    /// PxTriangleMesh pointer on success.
physx_PxTriangleMesh*      physx_phys_PxCreateTriangleMesh(physx_PxCookingParams const* params, physx_PxTriangleMeshDesc const* desc, physx_PxInsertionCallback* insertionCallback, int32_t* condition);

    /// Cooks a triangle mesh. The results are written to the stream.
    ///
    /// To create a triangle mesh object it is necessary to first 'cook' the mesh data into
    /// a form which allows the SDK to perform efficient collision detection.
    ///
    /// PxCookTriangleMesh() allows a mesh description to be cooked into a binary stream
    /// suitable for loading and performing collision detection at runtime.
    ///
    /// true on success
bool      physx_phys_PxCookTriangleMesh(physx_PxCookingParams const* params, physx_PxTriangleMeshDesc const* desc, physx_PxOutputStream* stream, int32_t* condition);

physx_PxDefaultMemoryOutputStream*      physx_PxDefaultMemoryOutputStream_new_alloc(physx_PxAllocatorCallback* allocator);

void      physx_PxDefaultMemoryOutputStream_delete(physx_PxDefaultMemoryOutputStream* self_);

uint32_t      physx_PxDefaultMemoryOutputStream_write_mut(physx_PxDefaultMemoryOutputStream* self_, void const* src, uint32_t count);

uint32_t      physx_PxDefaultMemoryOutputStream_getSize(physx_PxDefaultMemoryOutputStream const* self_);

uint8_t*      physx_PxDefaultMemoryOutputStream_getData(physx_PxDefaultMemoryOutputStream const* self_);

physx_PxDefaultMemoryInputData*      physx_PxDefaultMemoryInputData_new_alloc(uint8_t* data, uint32_t length);

uint32_t      physx_PxDefaultMemoryInputData_read_mut(physx_PxDefaultMemoryInputData* self_, void* dest, uint32_t count);

uint32_t      physx_PxDefaultMemoryInputData_getLength(physx_PxDefaultMemoryInputData const* self_);

void      physx_PxDefaultMemoryInputData_seek_mut(physx_PxDefaultMemoryInputData* self_, uint32_t pos);

uint32_t      physx_PxDefaultMemoryInputData_tell(physx_PxDefaultMemoryInputData const* self_);

physx_PxDefaultFileOutputStream*      physx_PxDefaultFileOutputStream_new_alloc(char const* name);

void      physx_PxDefaultFileOutputStream_delete(physx_PxDefaultFileOutputStream* self_);

uint32_t      physx_PxDefaultFileOutputStream_write_mut(physx_PxDefaultFileOutputStream* self_, void const* src, uint32_t count);

bool      physx_PxDefaultFileOutputStream_isValid_mut(physx_PxDefaultFileOutputStream* self_);

physx_PxDefaultFileInputData*      physx_PxDefaultFileInputData_new_alloc(char const* name);

void      physx_PxDefaultFileInputData_delete(physx_PxDefaultFileInputData* self_);

uint32_t      physx_PxDefaultFileInputData_read_mut(physx_PxDefaultFileInputData* self_, void* dest, uint32_t count);

void      physx_PxDefaultFileInputData_seek_mut(physx_PxDefaultFileInputData* self_, uint32_t pos);

uint32_t      physx_PxDefaultFileInputData_tell(physx_PxDefaultFileInputData const* self_);

uint32_t      physx_PxDefaultFileInputData_getLength(physx_PxDefaultFileInputData const* self_);

bool      physx_PxDefaultFileInputData_isValid(physx_PxDefaultFileInputData const* self_);

void*      physx_phys_platformAlignedAlloc(size_t size);

void      physx_phys_platformAlignedFree(void* ptr);

void*      physx_PxDefaultAllocator_allocate_mut(physx_PxDefaultAllocator* self_, size_t size, char const* anon_param1, char const* anon_param2, int32_t anon_param3);

void      physx_PxDefaultAllocator_deallocate_mut(physx_PxDefaultAllocator* self_, void* ptr);

void      physx_PxDefaultAllocator_delete(physx_PxDefaultAllocator* self_);

    /// Set the actors for this joint.
    ///
    /// An actor may be NULL to indicate the world frame. At most one of the actors may be NULL.
void      physx_PxJoint_setActors_mut(physx_PxJoint* self_, physx_PxRigidActor* actor0, physx_PxRigidActor* actor1);

    /// Get the actors for this joint.
void      physx_PxJoint_getActors(physx_PxJoint const* self_, physx_PxRigidActor** actor0, physx_PxRigidActor** actor1);

    /// Set the joint local pose for an actor.
    ///
    /// This is the relative pose which locates the joint frame relative to the actor.
void      physx_PxJoint_setLocalPose_mut(physx_PxJoint* self_, int32_t actor, physx_PxTransform const* localPose);

    /// get the joint local pose for an actor.
    ///
    /// return the local pose for this joint
physx_PxTransform      physx_PxJoint_getLocalPose(physx_PxJoint const* self_, int32_t actor);

    /// get the relative pose for this joint
    ///
    /// This function returns the pose of the joint frame of actor1 relative to actor0
physx_PxTransform      physx_PxJoint_getRelativeTransform(physx_PxJoint const* self_);

    /// get the relative linear velocity of the joint
    ///
    /// This function returns the linear velocity of the origin of the constraint frame of actor1, relative to the origin of the constraint
    /// frame of actor0. The value is returned in the constraint frame of actor0
physx_PxVec3      physx_PxJoint_getRelativeLinearVelocity(physx_PxJoint const* self_);

    /// get the relative angular velocity of the joint
    ///
    /// This function returns the angular velocity of  actor1 relative to actor0. The value is returned in the constraint frame of actor0
physx_PxVec3      physx_PxJoint_getRelativeAngularVelocity(physx_PxJoint const* self_);

    /// set the break force for this joint.
    ///
    /// if the constraint force or torque on the joint exceeds the specified values, the joint will break,
    /// at which point it will not constrain the two actors and the flag PxConstraintFlag::eBROKEN will be set. The
    /// force and torque are measured in the joint frame of the first actor
void      physx_PxJoint_setBreakForce_mut(physx_PxJoint* self_, float force, float torque);

    /// get the break force for this joint.
void      physx_PxJoint_getBreakForce(physx_PxJoint const* self_, float* force, float* torque);

    /// set the constraint flags for this joint.
void      physx_PxJoint_setConstraintFlags_mut(physx_PxJoint* self_, uint16_t flags);

    /// set a constraint flags for this joint to a specified value.
void      physx_PxJoint_setConstraintFlag_mut(physx_PxJoint* self_, int32_t flag, bool value);

    /// get the constraint flags for this joint.
    ///
    /// the constraint flags
uint16_t      physx_PxJoint_getConstraintFlags(physx_PxJoint const* self_);

    /// set the inverse mass scale for actor0.
void      physx_PxJoint_setInvMassScale0_mut(physx_PxJoint* self_, float invMassScale);

    /// get the inverse mass scale for actor0.
    ///
    /// inverse mass scale for actor0
float      physx_PxJoint_getInvMassScale0(physx_PxJoint const* self_);

    /// set the inverse inertia scale for actor0.
void      physx_PxJoint_setInvInertiaScale0_mut(physx_PxJoint* self_, float invInertiaScale);

    /// get the inverse inertia scale for actor0.
    ///
    /// inverse inertia scale for actor0
float      physx_PxJoint_getInvInertiaScale0(physx_PxJoint const* self_);

    /// set the inverse mass scale for actor1.
void      physx_PxJoint_setInvMassScale1_mut(physx_PxJoint* self_, float invMassScale);

    /// get the inverse mass scale for actor1.
    ///
    /// inverse mass scale for actor1
float      physx_PxJoint_getInvMassScale1(physx_PxJoint const* self_);

    /// set the inverse inertia scale for actor1.
void      physx_PxJoint_setInvInertiaScale1_mut(physx_PxJoint* self_, float invInertiaScale);

    /// get the inverse inertia scale for actor1.
    ///
    /// inverse inertia scale for actor1
float      physx_PxJoint_getInvInertiaScale1(physx_PxJoint const* self_);

    /// Retrieves the PxConstraint corresponding to this joint.
    ///
    /// This can be used to determine, among other things, the force applied at the joint.
    ///
    /// the constraint
physx_PxConstraint*      physx_PxJoint_getConstraint(physx_PxJoint const* self_);

    /// Sets a name string for the object that can be retrieved with getName().
    ///
    /// This is for debugging and is not used by the SDK. The string is not copied by the SDK,
    /// only the pointer is stored.
void      physx_PxJoint_setName_mut(physx_PxJoint* self_, char const* name);

    /// Retrieves the name string set with setName().
    ///
    /// Name string associated with object.
char const*      physx_PxJoint_getName(physx_PxJoint const* self_);

    /// Deletes the joint.
    ///
    /// This call does not wake up the connected rigid bodies.
void      physx_PxJoint_release_mut(physx_PxJoint* self_);

    /// Retrieves the scene which this joint belongs to.
    ///
    /// Owner Scene. NULL if not part of a scene.
physx_PxScene*      physx_PxJoint_getScene(physx_PxJoint const* self_);

    /// Put class meta data in stream, used for serialization
void      physx_PxJoint_getBinaryMetaData(physx_PxOutputStream* stream);

physx_PxSpring      physx_PxSpring_new(float stiffness_, float damping_);

    /// Helper function to setup a joint's global frame
    ///
    /// This replaces the following functions from previous SDK versions:
    ///
    /// void NxJointDesc::setGlobalAnchor(const NxVec3
    /// &
    /// wsAnchor);
    /// void NxJointDesc::setGlobalAxis(const NxVec3
    /// &
    /// wsAxis);
    ///
    /// The function sets the joint's localPose using world-space input parameters.
void      physx_phys_PxSetJointGlobalFrame(physx_PxJoint* joint, physx_PxVec3 const* wsAnchor, physx_PxVec3 const* wsAxis);

    /// Create a distance Joint.
physx_PxDistanceJoint*      physx_phys_PxDistanceJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// Return the current distance of the joint
float      physx_PxDistanceJoint_getDistance(physx_PxDistanceJoint const* self_);

    /// Set the allowed minimum distance for the joint.
    ///
    /// The minimum distance must be no more than the maximum distance
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
void      physx_PxDistanceJoint_setMinDistance_mut(physx_PxDistanceJoint* self_, float distance);

    /// Get the allowed minimum distance for the joint.
    ///
    /// the allowed minimum distance
float      physx_PxDistanceJoint_getMinDistance(physx_PxDistanceJoint const* self_);

    /// Set the allowed maximum distance for the joint.
    ///
    /// The maximum distance must be no less than the minimum distance.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
void      physx_PxDistanceJoint_setMaxDistance_mut(physx_PxDistanceJoint* self_, float distance);

    /// Get the allowed maximum distance for the joint.
    ///
    /// the allowed maximum distance
float      physx_PxDistanceJoint_getMaxDistance(physx_PxDistanceJoint const* self_);

    /// Set the error tolerance of the joint.
void      physx_PxDistanceJoint_setTolerance_mut(physx_PxDistanceJoint* self_, float tolerance);

    /// Get the error tolerance of the joint.
    ///
    /// the distance beyond the joint's [min, max] range before the joint becomes active.
    ///
    /// Default
    /// 0.25f * PxTolerancesScale::length
    /// Range
    /// (0, PX_MAX_F32)
    ///
    /// This value should be used to ensure that if the minimum distance is zero and the
    /// spring function is in use, the rest length of the spring is non-zero.
float      physx_PxDistanceJoint_getTolerance(physx_PxDistanceJoint const* self_);

    /// Set the strength of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
void      physx_PxDistanceJoint_setStiffness_mut(physx_PxDistanceJoint* self_, float stiffness);

    /// Get the strength of the joint spring.
    ///
    /// stiffness the spring strength of the joint
float      physx_PxDistanceJoint_getStiffness(physx_PxDistanceJoint const* self_);

    /// Set the damping of the joint spring.
    ///
    /// The spring is used if enabled, and the distance exceeds the range [min-error, max+error].
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
void      physx_PxDistanceJoint_setDamping_mut(physx_PxDistanceJoint* self_, float damping);

    /// Get the damping of the joint spring.
    ///
    /// the degree of damping of the joint spring of the joint
float      physx_PxDistanceJoint_getDamping(physx_PxDistanceJoint const* self_);

    /// Set the contact distance for the min
    /// &
    /// max distance limits.
    ///
    /// This is similar to the PxJointLimitParameters::contactDistance parameter for regular limits.
    ///
    /// The two most common values are 0 and infinite. Infinite means the internal constraints are
    /// always created, resulting in the best simulation quality but slower performance. Zero means
    /// the internal constraints are only created when the limits are violated, resulting in best
    /// performance but worse simulation quality.
    ///
    /// Default
    /// 0.0f
    /// Range
    /// [0, PX_MAX_F32)
void      physx_PxDistanceJoint_setContactDistance_mut(physx_PxDistanceJoint* self_, float contactDistance);

    /// Get the contact distance.
    ///
    /// the contact distance
float      physx_PxDistanceJoint_getContactDistance(physx_PxDistanceJoint const* self_);

    /// Set the flags specific to the Distance Joint.
    ///
    /// Default
    /// PxDistanceJointFlag::eMAX_DISTANCE_ENABLED
void      physx_PxDistanceJoint_setDistanceJointFlags_mut(physx_PxDistanceJoint* self_, uint16_t flags);

    /// Set a single flag specific to a Distance Joint to true or false.
void      physx_PxDistanceJoint_setDistanceJointFlag_mut(physx_PxDistanceJoint* self_, int32_t flag, bool value);

    /// Get the flags specific to the Distance Joint.
    ///
    /// the joint flags
uint16_t      physx_PxDistanceJoint_getDistanceJointFlags(physx_PxDistanceJoint const* self_);

    /// Returns string name of PxDistanceJoint, used for serialization
char const*      physx_PxDistanceJoint_getConcreteTypeName(physx_PxDistanceJoint const* self_);

    /// Create a distance Joint.
physx_PxContactJoint*      physx_phys_PxContactJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

physx_PxJacobianRow      physx_PxJacobianRow_new();

physx_PxJacobianRow      physx_PxJacobianRow_new_1(physx_PxVec3 const* lin0, physx_PxVec3 const* lin1, physx_PxVec3 const* ang0, physx_PxVec3 const* ang1);

    /// Set the current contact of the joint
void      physx_PxContactJoint_setContact_mut(physx_PxContactJoint* self_, physx_PxVec3 const* contact);

    /// Set the current contact normal of the joint
void      physx_PxContactJoint_setContactNormal_mut(physx_PxContactJoint* self_, physx_PxVec3 const* contactNormal);

    /// Set the current penetration of the joint
void      physx_PxContactJoint_setPenetration_mut(physx_PxContactJoint* self_, float penetration);

    /// Return the current contact of the joint
physx_PxVec3      physx_PxContactJoint_getContact(physx_PxContactJoint const* self_);

    /// Return the current contact normal of the joint
physx_PxVec3      physx_PxContactJoint_getContactNormal(physx_PxContactJoint const* self_);

    /// Return the current penetration value of the joint
float      physx_PxContactJoint_getPenetration(physx_PxContactJoint const* self_);

float      physx_PxContactJoint_getRestitution(physx_PxContactJoint const* self_);

void      physx_PxContactJoint_setRestitution_mut(physx_PxContactJoint* self_, float restitution);

float      physx_PxContactJoint_getBounceThreshold(physx_PxContactJoint const* self_);

void      physx_PxContactJoint_setBounceThreshold_mut(physx_PxContactJoint* self_, float bounceThreshold);

    /// Returns string name of PxContactJoint, used for serialization
char const*      physx_PxContactJoint_getConcreteTypeName(physx_PxContactJoint const* self_);

void      physx_PxContactJoint_computeJacobians(physx_PxContactJoint const* self_, physx_PxJacobianRow* jacobian);

uint32_t      physx_PxContactJoint_getNbJacobianRows(physx_PxContactJoint const* self_);

    /// Create a fixed joint.
physx_PxFixedJoint*      physx_phys_PxFixedJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// Returns string name of PxFixedJoint, used for serialization
char const*      physx_PxFixedJoint_getConcreteTypeName(physx_PxFixedJoint const* self_);

physx_PxJointLimitParameters*      physx_PxJointLimitParameters_new_alloc();

    /// Returns true if the current settings are valid.
    ///
    /// true if the current settings are valid
bool      physx_PxJointLimitParameters_isValid(physx_PxJointLimitParameters const* self_);

bool      physx_PxJointLimitParameters_isSoft(physx_PxJointLimitParameters const* self_);

    /// construct a linear hard limit
physx_PxJointLinearLimit      physx_PxJointLinearLimit_new(physx_PxTolerancesScale const* scale, float extent, float contactDist_deprecated);

    /// construct a linear soft limit
physx_PxJointLinearLimit      physx_PxJointLinearLimit_new_1(float extent, physx_PxSpring const* spring);

    /// Returns true if the limit is valid
    ///
    /// true if the current settings are valid
bool      physx_PxJointLinearLimit_isValid(physx_PxJointLinearLimit const* self_);

void      physx_PxJointLinearLimit_delete(physx_PxJointLinearLimit* self_);

    /// Construct a linear hard limit pair. The lower distance value must be less than the upper distance value.
physx_PxJointLinearLimitPair      physx_PxJointLinearLimitPair_new(physx_PxTolerancesScale const* scale, float lowerLimit, float upperLimit, float contactDist_deprecated);

    /// construct a linear soft limit pair
physx_PxJointLinearLimitPair      physx_PxJointLinearLimitPair_new_1(float lowerLimit, float upperLimit, physx_PxSpring const* spring);

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
bool      physx_PxJointLinearLimitPair_isValid(physx_PxJointLinearLimitPair const* self_);

void      physx_PxJointLinearLimitPair_delete(physx_PxJointLinearLimitPair* self_);

    /// construct an angular hard limit pair.
    ///
    /// The lower value must be less than the upper value.
physx_PxJointAngularLimitPair      physx_PxJointAngularLimitPair_new(float lowerLimit, float upperLimit, float contactDist_deprecated);

    /// construct an angular soft limit pair.
    ///
    /// The lower value must be less than the upper value.
physx_PxJointAngularLimitPair      physx_PxJointAngularLimitPair_new_1(float lowerLimit, float upperLimit, physx_PxSpring const* spring);

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
bool      physx_PxJointAngularLimitPair_isValid(physx_PxJointAngularLimitPair const* self_);

void      physx_PxJointAngularLimitPair_delete(physx_PxJointAngularLimitPair* self_);

    /// Construct a cone hard limit.
physx_PxJointLimitCone      physx_PxJointLimitCone_new(float yLimitAngle, float zLimitAngle, float contactDist_deprecated);

    /// Construct a cone soft limit.
physx_PxJointLimitCone      physx_PxJointLimitCone_new_1(float yLimitAngle, float zLimitAngle, physx_PxSpring const* spring);

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
bool      physx_PxJointLimitCone_isValid(physx_PxJointLimitCone const* self_);

void      physx_PxJointLimitCone_delete(physx_PxJointLimitCone* self_);

    /// Construct a pyramid hard limit.
physx_PxJointLimitPyramid      physx_PxJointLimitPyramid_new(float yLimitAngleMin, float yLimitAngleMax, float zLimitAngleMin, float zLimitAngleMax, float contactDist_deprecated);

    /// Construct a pyramid soft limit.
physx_PxJointLimitPyramid      physx_PxJointLimitPyramid_new_1(float yLimitAngleMin, float yLimitAngleMax, float zLimitAngleMin, float zLimitAngleMax, physx_PxSpring const* spring);

    /// Returns true if the limit is valid.
    ///
    /// true if the current settings are valid
bool      physx_PxJointLimitPyramid_isValid(physx_PxJointLimitPyramid const* self_);

void      physx_PxJointLimitPyramid_delete(physx_PxJointLimitPyramid* self_);

    /// Create a prismatic joint.
physx_PxPrismaticJoint*      physx_phys_PxPrismaticJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// returns the displacement of the joint along its axis.
float      physx_PxPrismaticJoint_getPosition(physx_PxPrismaticJoint const* self_);

    /// returns the velocity of the joint along its axis
float      physx_PxPrismaticJoint_getVelocity(physx_PxPrismaticJoint const* self_);

    /// sets the joint limit  parameters.
    ///
    /// The limit range is [-PX_MAX_F32, PX_MAX_F32], but note that the width of the limit (upper-lower) must also be
    /// a valid float.
void      physx_PxPrismaticJoint_setLimit_mut(physx_PxPrismaticJoint* self_, physx_PxJointLinearLimitPair const* anon_param0);

    /// gets the joint limit  parameters.
physx_PxJointLinearLimitPair      physx_PxPrismaticJoint_getLimit(physx_PxPrismaticJoint const* self_);

    /// Set the flags specific to the Prismatic Joint.
    ///
    /// Default
    /// PxPrismaticJointFlags(0)
void      physx_PxPrismaticJoint_setPrismaticJointFlags_mut(physx_PxPrismaticJoint* self_, uint16_t flags);

    /// Set a single flag specific to a Prismatic Joint to true or false.
void      physx_PxPrismaticJoint_setPrismaticJointFlag_mut(physx_PxPrismaticJoint* self_, int32_t flag, bool value);

    /// Get the flags specific to the Prismatic Joint.
    ///
    /// the joint flags
uint16_t      physx_PxPrismaticJoint_getPrismaticJointFlags(physx_PxPrismaticJoint const* self_);

    /// Returns string name of PxPrismaticJoint, used for serialization
char const*      physx_PxPrismaticJoint_getConcreteTypeName(physx_PxPrismaticJoint const* self_);

    /// Create a revolute joint.
physx_PxRevoluteJoint*      physx_phys_PxRevoluteJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// return the angle of the joint, in the range (-2*Pi, 2*Pi]
float      physx_PxRevoluteJoint_getAngle(physx_PxRevoluteJoint const* self_);

    /// return the velocity of the joint
float      physx_PxRevoluteJoint_getVelocity(physx_PxRevoluteJoint const* self_);

    /// set the joint limit parameters.
    ///
    /// The limit is activated using the flag PxRevoluteJointFlag::eLIMIT_ENABLED
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
void      physx_PxRevoluteJoint_setLimit_mut(physx_PxRevoluteJoint* self_, physx_PxJointAngularLimitPair const* limits);

    /// get the joint limit parameters.
    ///
    /// the joint limit parameters
physx_PxJointAngularLimitPair      physx_PxRevoluteJoint_getLimit(physx_PxRevoluteJoint const* self_);

    /// set the target velocity for the drive model.
    ///
    /// The motor will only be able to reach this velocity if the maxForce is sufficiently large.
    /// If the joint is spinning faster than this velocity, the motor will actually try to brake
    /// (see PxRevoluteJointFlag::eDRIVE_FREESPIN.)
    ///
    /// The sign of this variable determines the rotation direction, with positive values going
    /// the same way as positive joint angles. Setting a very large target velocity may cause
    /// undesirable results.
    ///
    /// Range:
    /// (-PX_MAX_F32, PX_MAX_F32)
    /// Default:
    /// 0.0
void      physx_PxRevoluteJoint_setDriveVelocity_mut(physx_PxRevoluteJoint* self_, float velocity, bool autowake);

    /// gets the target velocity for the drive model.
    ///
    /// the drive target velocity
float      physx_PxRevoluteJoint_getDriveVelocity(physx_PxRevoluteJoint const* self_);

    /// sets the maximum torque the drive can exert.
    ///
    /// The value set here may be used either as an impulse limit or a force limit, depending on the flag PxConstraintFlag::eDRIVE_LIMITS_ARE_FORCES
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// PX_MAX_F32
void      physx_PxRevoluteJoint_setDriveForceLimit_mut(physx_PxRevoluteJoint* self_, float limit);

    /// gets the maximum torque the drive can exert.
    ///
    /// the torque limit
float      physx_PxRevoluteJoint_getDriveForceLimit(physx_PxRevoluteJoint const* self_);

    /// sets the gear ratio for the drive.
    ///
    /// When setting up the drive constraint, the velocity of the first actor is scaled by this value, and its response to drive torque is scaled down.
    /// So if the drive target velocity is zero, the second actor will be driven to the velocity of the first scaled by the gear ratio
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1.0
void      physx_PxRevoluteJoint_setDriveGearRatio_mut(physx_PxRevoluteJoint* self_, float ratio);

    /// gets the gear ratio.
    ///
    /// the drive gear ratio
float      physx_PxRevoluteJoint_getDriveGearRatio(physx_PxRevoluteJoint const* self_);

    /// sets the flags specific to the Revolute Joint.
    ///
    /// Default
    /// PxRevoluteJointFlags(0)
void      physx_PxRevoluteJoint_setRevoluteJointFlags_mut(physx_PxRevoluteJoint* self_, uint16_t flags);

    /// sets a single flag specific to a Revolute Joint.
void      physx_PxRevoluteJoint_setRevoluteJointFlag_mut(physx_PxRevoluteJoint* self_, int32_t flag, bool value);

    /// gets the flags specific to the Revolute Joint.
    ///
    /// the joint flags
uint16_t      physx_PxRevoluteJoint_getRevoluteJointFlags(physx_PxRevoluteJoint const* self_);

    /// Returns string name of PxRevoluteJoint, used for serialization
char const*      physx_PxRevoluteJoint_getConcreteTypeName(physx_PxRevoluteJoint const* self_);

    /// Create a spherical joint.
physx_PxSphericalJoint*      physx_phys_PxSphericalJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// Set the limit cone.
    ///
    /// If enabled, the limit cone will constrain the angular movement of the joint to lie
    /// within an elliptical cone.
    ///
    /// the limit cone
physx_PxJointLimitCone      physx_PxSphericalJoint_getLimitCone(physx_PxSphericalJoint const* self_);

    /// Get the limit cone.
void      physx_PxSphericalJoint_setLimitCone_mut(physx_PxSphericalJoint* self_, physx_PxJointLimitCone const* limit);

    /// get the swing angle of the joint from the Y axis
float      physx_PxSphericalJoint_getSwingYAngle(physx_PxSphericalJoint const* self_);

    /// get the swing angle of the joint from the Z axis
float      physx_PxSphericalJoint_getSwingZAngle(physx_PxSphericalJoint const* self_);

    /// Set the flags specific to the Spherical Joint.
    ///
    /// Default
    /// PxSphericalJointFlags(0)
void      physx_PxSphericalJoint_setSphericalJointFlags_mut(physx_PxSphericalJoint* self_, uint16_t flags);

    /// Set a single flag specific to a Spherical Joint to true or false.
void      physx_PxSphericalJoint_setSphericalJointFlag_mut(physx_PxSphericalJoint* self_, int32_t flag, bool value);

    /// Get the flags specific to the Spherical Joint.
    ///
    /// the joint flags
uint16_t      physx_PxSphericalJoint_getSphericalJointFlags(physx_PxSphericalJoint const* self_);

    /// Returns string name of PxSphericalJoint, used for serialization
char const*      physx_PxSphericalJoint_getConcreteTypeName(physx_PxSphericalJoint const* self_);

    /// Create a D6 joint.
physx_PxD6Joint*      physx_phys_PxD6JointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// default constructor for PxD6JointDrive.
physx_PxD6JointDrive      physx_PxD6JointDrive_new();

    /// constructor a PxD6JointDrive.
physx_PxD6JointDrive      physx_PxD6JointDrive_new_1(float driveStiffness, float driveDamping, float driveForceLimit, bool isAcceleration);

    /// returns true if the drive is valid
bool      physx_PxD6JointDrive_isValid(physx_PxD6JointDrive const* self_);

    /// Set the motion type around the specified axis.
    ///
    /// Each axis may independently specify that the degree of freedom is locked (blocking relative movement
    /// along or around this axis), limited by the corresponding limit, or free.
    ///
    /// Default:
    /// all degrees of freedom are locked
void      physx_PxD6Joint_setMotion_mut(physx_PxD6Joint* self_, int32_t axis, int32_t type_);

    /// Get the motion type around the specified axis.
    ///
    /// the motion type around the specified axis
int32_t      physx_PxD6Joint_getMotion(physx_PxD6Joint const* self_, int32_t axis);

    /// get the twist angle of the joint, in the range (-2*Pi, 2*Pi]
float      physx_PxD6Joint_getTwistAngle(physx_PxD6Joint const* self_);

    /// get the swing angle of the joint from the Y axis
float      physx_PxD6Joint_getSwingYAngle(physx_PxD6Joint const* self_);

    /// get the swing angle of the joint from the Z axis
float      physx_PxD6Joint_getSwingZAngle(physx_PxD6Joint const* self_);

    /// Set the distance limit for the joint.
    ///
    /// A single limit constraints all linear limited degrees of freedom, forming a linear, circular
    /// or spherical constraint on motion depending on the number of limited degrees. This is similar
    /// to a distance limit.
void      physx_PxD6Joint_setDistanceLimit_mut(physx_PxD6Joint* self_, physx_PxJointLinearLimit const* limit);

    /// Get the distance limit for the joint.
    ///
    /// the distance limit structure
physx_PxJointLinearLimit      physx_PxD6Joint_getDistanceLimit(physx_PxD6Joint const* self_);

    /// Set the linear limit for a given linear axis.
    ///
    /// This function extends the previous setDistanceLimit call with the following features:
    /// - there can be a different limit for each linear axis
    /// - each limit is defined by two values, i.e. it can now be asymmetric
    ///
    /// This can be used to create prismatic joints similar to PxPrismaticJoint, or point-in-quad joints,
    /// or point-in-box joints.
void      physx_PxD6Joint_setLinearLimit_mut(physx_PxD6Joint* self_, int32_t axis, physx_PxJointLinearLimitPair const* limit);

    /// Get the linear limit for a given linear axis.
    ///
    /// the linear limit pair structure from desired axis
physx_PxJointLinearLimitPair      physx_PxD6Joint_getLinearLimit(physx_PxD6Joint const* self_, int32_t axis);

    /// Set the twist limit for the joint.
    ///
    /// The twist limit controls the range of motion around the twist axis.
    ///
    /// The limit angle range is (-2*Pi, 2*Pi).
void      physx_PxD6Joint_setTwistLimit_mut(physx_PxD6Joint* self_, physx_PxJointAngularLimitPair const* limit);

    /// Get the twist limit for the joint.
    ///
    /// the twist limit structure
physx_PxJointAngularLimitPair      physx_PxD6Joint_getTwistLimit(physx_PxD6Joint const* self_);

    /// Set the swing cone limit for the joint.
    ///
    /// The cone limit is used if either or both swing axes are limited. The extents are
    /// symmetrical and measured in the frame of the parent. If only one swing degree of freedom
    /// is limited, the corresponding value from the cone limit defines the limit range.
void      physx_PxD6Joint_setSwingLimit_mut(physx_PxD6Joint* self_, physx_PxJointLimitCone const* limit);

    /// Get the cone limit for the joint.
    ///
    /// the swing limit structure
physx_PxJointLimitCone      physx_PxD6Joint_getSwingLimit(physx_PxD6Joint const* self_);

    /// Set a pyramidal swing limit for the joint.
    ///
    /// The pyramid limits will only be used in the following cases:
    /// - both swing Y and Z are limited. The limit shape is then a pyramid.
    /// - Y is limited and Z is locked, or vice versa. The limit shape is an asymmetric angular section, similar to
    /// what is supported for the twist axis.
    /// The remaining cases (Y limited and Z is free, or vice versa) are not supported.
void      physx_PxD6Joint_setPyramidSwingLimit_mut(physx_PxD6Joint* self_, physx_PxJointLimitPyramid const* limit);

    /// Get the pyramidal swing limit for the joint.
    ///
    /// the swing limit structure
physx_PxJointLimitPyramid      physx_PxD6Joint_getPyramidSwingLimit(physx_PxD6Joint const* self_);

    /// Set the drive parameters for the specified drive type.
    ///
    /// Default
    /// The default drive spring and damping values are zero, the force limit is zero, and no flags are set.
void      physx_PxD6Joint_setDrive_mut(physx_PxD6Joint* self_, int32_t index, physx_PxD6JointDrive const* drive);

    /// Get the drive parameters for the specified drive type.
physx_PxD6JointDrive      physx_PxD6Joint_getDrive(physx_PxD6Joint const* self_, int32_t index);

    /// Set the drive goal pose
    ///
    /// The goal is relative to the constraint frame of actor[0]
    ///
    /// Default
    /// the identity transform
void      physx_PxD6Joint_setDrivePosition_mut(physx_PxD6Joint* self_, physx_PxTransform const* pose, bool autowake);

    /// Get the drive goal pose.
physx_PxTransform      physx_PxD6Joint_getDrivePosition(physx_PxD6Joint const* self_);

    /// Set the target goal velocity for drive.
    ///
    /// The velocity is measured in the constraint frame of actor[0]
void      physx_PxD6Joint_setDriveVelocity_mut(physx_PxD6Joint* self_, physx_PxVec3 const* linear, physx_PxVec3 const* angular, bool autowake);

    /// Get the target goal velocity for joint drive.
void      physx_PxD6Joint_getDriveVelocity(physx_PxD6Joint const* self_, physx_PxVec3* linear, physx_PxVec3* angular);

    /// Set the linear tolerance threshold for projection. Projection is enabled if PxConstraintFlag::ePROJECTION
    /// is set for the joint.
    ///
    /// If the joint separates by more than this distance along its locked degrees of freedom, the solver
    /// will move the bodies to close the distance.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    ///
    /// Sometimes it is not possible to project (for example when the joints form a cycle).
    ///
    /// Range:
    /// [0, PX_MAX_F32)
    /// Default:
    /// 1e10f
void      physx_PxD6Joint_setProjectionLinearTolerance_mut(physx_PxD6Joint* self_, float tolerance);

    /// Get the linear tolerance threshold for projection.
    ///
    /// the linear tolerance threshold
float      physx_PxD6Joint_getProjectionLinearTolerance(physx_PxD6Joint const* self_);

    /// Set the angular tolerance threshold for projection. Projection is enabled if
    /// PxConstraintFlag::ePROJECTION is set for the joint.
    ///
    /// If the joint deviates by more than this angle around its locked angular degrees of freedom,
    /// the solver will move the bodies to close the angle.
    ///
    /// Setting a very small tolerance may result in simulation jitter or other artifacts.
    ///
    /// Sometimes it is not possible to project (for example when the joints form a cycle).
    ///
    /// Range:
    /// [0,Pi]
    /// Default:
    /// Pi
    ///
    /// Angular projection is implemented only for the case of two or three locked angular degrees of freedom.
void      physx_PxD6Joint_setProjectionAngularTolerance_mut(physx_PxD6Joint* self_, float tolerance);

    /// Get the angular tolerance threshold for projection.
    ///
    /// tolerance the angular tolerance threshold in radians
float      physx_PxD6Joint_getProjectionAngularTolerance(physx_PxD6Joint const* self_);

    /// Returns string name of PxD6Joint, used for serialization
char const*      physx_PxD6Joint_getConcreteTypeName(physx_PxD6Joint const* self_);

    /// Create a gear Joint.
physx_PxGearJoint*      physx_phys_PxGearJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// Set the hinge/revolute joints connected by the gear joint.
    ///
    /// The passed joints can be either PxRevoluteJoint, PxD6Joint or PxArticulationJointReducedCoordinate.
    /// The joints must define degrees of freedom around the twist axis. They cannot be null.
    ///
    /// Note that these joints are only used to compute the positional error correction term,
    /// used to adjust potential drift between jointed actors. The gear joint can run without
    /// calling this function, but in that case some visible overlap may develop over time between
    /// the teeth of the gear meshes.
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
bool      physx_PxGearJoint_setHinges_mut(physx_PxGearJoint* self_, physx_PxBase const* hinge0, physx_PxBase const* hinge1);

    /// Set the desired gear ratio.
    ///
    /// For two gears with n0 and n1 teeth respectively, the gear ratio is n0/n1.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
void      physx_PxGearJoint_setGearRatio_mut(physx_PxGearJoint* self_, float ratio);

    /// Get the gear ratio.
    ///
    /// Current ratio
float      physx_PxGearJoint_getGearRatio(physx_PxGearJoint const* self_);

char const*      physx_PxGearJoint_getConcreteTypeName(physx_PxGearJoint const* self_);

    /// Create a rack
    /// &
    /// pinion Joint.
physx_PxRackAndPinionJoint*      physx_phys_PxRackAndPinionJointCreate(physx_PxPhysics* physics, physx_PxRigidActor* actor0, physx_PxTransform const* localFrame0, physx_PxRigidActor* actor1, physx_PxTransform const* localFrame1);

    /// Set the hinge
    /// &
    /// prismatic joints connected by the rack
    /// &
    /// pinion joint.
    ///
    /// The passed hinge joint can be either PxRevoluteJoint, PxD6Joint or PxArticulationJointReducedCoordinate. It cannot be null.
    /// The passed prismatic joint can be either PxPrismaticJoint or PxD6Joint. It cannot be null.
    ///
    /// Note that these joints are only used to compute the positional error correction term,
    /// used to adjust potential drift between jointed actors. The rack
    /// &
    /// pinion joint can run without
    /// calling this function, but in that case some visible overlap may develop over time between
    /// the teeth of the rack
    /// &
    /// pinion meshes.
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
bool      physx_PxRackAndPinionJoint_setJoints_mut(physx_PxRackAndPinionJoint* self_, physx_PxBase const* hinge, physx_PxBase const* prismatic);

    /// Set the desired ratio directly.
    ///
    /// You may need to use a negative gear ratio if the joint frames of involved actors are not oriented in the same direction.
    ///
    /// Calling this function resets the internal positional error correction term.
void      physx_PxRackAndPinionJoint_setRatio_mut(physx_PxRackAndPinionJoint* self_, float ratio);

    /// Get the ratio.
    ///
    /// Current ratio
float      physx_PxRackAndPinionJoint_getRatio(physx_PxRackAndPinionJoint const* self_);

    /// Set the desired ratio indirectly.
    ///
    /// This is a simple helper function that computes the ratio from passed data:
    ///
    /// ratio = (PI*2*nbRackTeeth)/(rackLength*nbPinionTeeth)
    ///
    /// Calling this function resets the internal positional error correction term.
    ///
    /// true if success
bool      physx_PxRackAndPinionJoint_setData_mut(physx_PxRackAndPinionJoint* self_, uint32_t nbRackTeeth, uint32_t nbPinionTeeth, float rackLength);

char const*      physx_PxRackAndPinionJoint_getConcreteTypeName(physx_PxRackAndPinionJoint const* self_);

physx_PxGroupsMask*      physx_PxGroupsMask_new_alloc();

void      physx_PxGroupsMask_delete(physx_PxGroupsMask* self_);

    /// Implementation of a simple filter shader that emulates PhysX 2.8.x filtering
    ///
    /// This shader provides the following logic:
    ///
    /// If one of the two filter objects is a trigger, the pair is acccepted and [`PxPairFlag::eTRIGGER_DEFAULT`] will be used for trigger reports
    ///
    /// Else, if the filter mask logic (see further below) discards the pair it will be suppressed ([`PxFilterFlag::eSUPPRESS`])
    ///
    /// Else, the pair gets accepted and collision response gets enabled ([`PxPairFlag::eCONTACT_DEFAULT`])
    ///
    /// Filter mask logic:
    /// Given the two [`PxFilterData`] structures fd0 and fd1 of two collision objects, the pair passes the filter if the following
    /// conditions are met:
    ///
    /// 1) Collision groups of the pair are enabled
    /// 2) Collision filtering equation is satisfied
uint16_t      physx_phys_PxDefaultSimulationFilterShader(uint32_t attributes0, physx_PxFilterData filterData0, uint32_t attributes1, physx_PxFilterData filterData1, uint16_t* pairFlags, void const* constantBlock, uint32_t constantBlockSize);

    /// Determines if collision detection is performed between a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// True if the groups could collide
bool      physx_phys_PxGetGroupCollisionFlag(uint16_t group1, uint16_t group2);

    /// Specifies if collision should be performed by a pair of groups
    ///
    /// Collision group is an integer between 0 and 31.
void      physx_phys_PxSetGroupCollisionFlag(uint16_t group1, uint16_t group2, bool enable);

    /// Retrieves the value set with PxSetGroup()
    ///
    /// Collision group is an integer between 0 and 31.
    ///
    /// The collision group this actor belongs to
uint16_t      physx_phys_PxGetGroup(physx_PxActor const* actor);

    /// Sets which collision group this actor is part of
    ///
    /// Collision group is an integer between 0 and 31.
void      physx_phys_PxSetGroup(physx_PxActor* actor, uint16_t collisionGroup);

    /// Retrieves filtering operation. See comments for PxGroupsMask
void      physx_phys_PxGetFilterOps(int32_t* op0, int32_t* op1, int32_t* op2);

    /// Setups filtering operations. See comments for PxGroupsMask
void      physx_phys_PxSetFilterOps(int32_t const* op0, int32_t const* op1, int32_t const* op2);

    /// Retrieves filtering's boolean value. See comments for PxGroupsMask
    ///
    /// flag Boolean value for filter.
bool      physx_phys_PxGetFilterBool();

    /// Setups filtering's boolean value. See comments for PxGroupsMask
void      physx_phys_PxSetFilterBool(bool enable);

    /// Gets filtering constant K0 and K1. See comments for PxGroupsMask
void      physx_phys_PxGetFilterConstants(physx_PxGroupsMask* c0, physx_PxGroupsMask* c1);

    /// Setups filtering's K0 and K1 value. See comments for PxGroupsMask
void      physx_phys_PxSetFilterConstants(physx_PxGroupsMask const* c0, physx_PxGroupsMask const* c1);

    /// Gets 64-bit mask used for collision filtering. See comments for PxGroupsMask
    ///
    /// The group mask for the actor.
physx_PxGroupsMask      physx_phys_PxGetGroupsMask(physx_PxActor const* actor);

    /// Sets 64-bit mask used for collision filtering. See comments for PxGroupsMask
void      physx_phys_PxSetGroupsMask(physx_PxActor* actor, physx_PxGroupsMask const* mask);

physx_PxDefaultErrorCallback*      physx_PxDefaultErrorCallback_new_alloc();

void      physx_PxDefaultErrorCallback_delete(physx_PxDefaultErrorCallback* self_);

void      physx_PxDefaultErrorCallback_reportError_mut(physx_PxDefaultErrorCallback* self_, int32_t code, char const* message, char const* file, int32_t line);

    /// Creates a new shape with default properties and a list of materials and adds it to the list of shapes of this actor.
    ///
    /// This is equivalent to the following
    ///
    /// ```cpp
    /// // reference count is 1
    /// PxShape* shape(...) = PxGetPhysics().createShape(...);
    /// // increments reference count
    /// actor->attachShape(shape);
    /// // releases user reference, leaving reference count at 1
    /// shape->release();
    /// ```
    ///
    /// As a consequence, detachShape() will result in the release of the last reference, and the shape will be deleted.
    ///
    /// The default shape flags to be set are: eVISUALIZATION, eSIMULATION_SHAPE, eSCENE_QUERY_SHAPE (see [`PxShapeFlag`]).
    /// Triangle mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Creating compounds with a very large number of shapes may adversely affect performance and stability.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// The newly created shape.
physx_PxShape*      physx_PxRigidActorExt_createExclusiveShape(physx_PxRigidActor* actor, physx_PxGeometry const* geometry, physx_PxMaterial* const* materials, uint16_t materialCount, uint8_t shapeFlags);

    /// Creates a new shape with default properties and a single material adds it to the list of shapes of this actor.
    ///
    /// This is equivalent to the following
    ///
    /// ```cpp
    /// // reference count is 1
    /// PxShape* shape(...) = PxGetPhysics().createShape(...);
    /// // increments reference count
    /// actor->attachShape(shape);
    /// // releases user reference, leaving reference count at 1
    /// shape->release();
    /// ```
    ///
    /// As a consequence, detachShape() will result in the release of the last reference, and the shape will be deleted.
    ///
    /// The default shape flags to be set are: eVISUALIZATION, eSIMULATION_SHAPE, eSCENE_QUERY_SHAPE (see [`PxShapeFlag`]).
    /// Triangle mesh, heightfield or plane geometry shapes configured as eSIMULATION_SHAPE are not supported for
    /// non-kinematic PxRigidDynamic instances.
    ///
    /// Creating compounds with a very large number of shapes may adversely affect performance and stability.
    ///
    /// Sleeping:
    /// Does
    /// NOT
    /// wake the actor up automatically.
    ///
    /// The newly created shape.
physx_PxShape*      physx_PxRigidActorExt_createExclusiveShape_1(physx_PxRigidActor* actor, physx_PxGeometry const* geometry, physx_PxMaterial const* material, uint8_t shapeFlags);

    /// Gets a list of bounds based on shapes in rigid actor. This list can be used to cook/create
    /// bounding volume hierarchy though PxCooking API.
physx_PxBounds3*      physx_PxRigidActorExt_getRigidActorShapeLocalBoundsList(physx_PxRigidActor const* actor, uint32_t* numBounds);

    /// Convenience function to create a PxBVH object from a PxRigidActor.
    ///
    /// The computed PxBVH can then be used in PxScene::addActor() or PxAggregate::addActor().
    /// After adding the actor
    /// &
    /// BVH to the scene/aggregate, release the PxBVH object by calling PxBVH::release().
    ///
    /// The PxBVH for this actor.
physx_PxBVH*      physx_PxRigidActorExt_createBVHFromActor(physx_PxPhysics* physics, physx_PxRigidActor const* actor);

    /// Default constructor.
physx_PxMassProperties      physx_PxMassProperties_new();

    /// Construct from individual elements.
physx_PxMassProperties      physx_PxMassProperties_new_1(float m, physx_PxMat33 const* inertiaT, physx_PxVec3 const* com);

    /// Compute mass properties based on a provided geometry structure.
    ///
    /// This constructor assumes the geometry has a density of 1. Mass and inertia tensor scale linearly with density.
physx_PxMassProperties      physx_PxMassProperties_new_2(physx_PxGeometry const* geometry);

    /// Translate the center of mass by a given vector and adjust the inertia tensor accordingly.
void      physx_PxMassProperties_translate_mut(physx_PxMassProperties* self_, physx_PxVec3 const* t);

    /// Get the entries of the diagonalized inertia tensor and the corresponding reference rotation.
    ///
    /// The entries of the diagonalized inertia tensor.
physx_PxVec3      physx_PxMassProperties_getMassSpaceInertia(physx_PxMat33 const* inertia, physx_PxQuat* massFrame);

    /// Translate an inertia tensor using the parallel axis theorem
    ///
    /// The translated inertia tensor.
physx_PxMat33      physx_PxMassProperties_translateInertia(physx_PxMat33 const* inertia, float mass, physx_PxVec3 const* t);

    /// Rotate an inertia tensor around the center of mass
    ///
    /// The rotated inertia tensor.
physx_PxMat33      physx_PxMassProperties_rotateInertia(physx_PxMat33 const* inertia, physx_PxQuat const* q);

    /// Non-uniform scaling of the inertia tensor
    ///
    /// The scaled inertia tensor.
physx_PxMat33      physx_PxMassProperties_scaleInertia(physx_PxMat33 const* inertia, physx_PxQuat const* scaleRotation, physx_PxVec3 const* scale);

    /// Sum up individual mass properties.
    ///
    /// The summed up mass properties.
physx_PxMassProperties      physx_PxMassProperties_sum(physx_PxMassProperties const* props, physx_PxTransform const* transforms, uint32_t count);

    /// Computation of mass properties for a rigid body actor
    ///
    /// To simulate a dynamic rigid actor, the SDK needs a mass and an inertia tensor.
    ///
    /// This method offers functionality to compute the necessary mass and inertia properties based on the shapes declared in
    /// the PxRigidBody descriptor and some additionally specified parameters. For each shape, the shape geometry,
    /// the shape positioning within the actor and the specified shape density are used to compute the body's mass and
    /// inertia properties.
    ///
    /// Shapes without PxShapeFlag::eSIMULATION_SHAPE set are ignored unless includeNonSimShapes is true.
    /// Shapes with plane, triangle mesh or heightfield geometry and PxShapeFlag::eSIMULATION_SHAPE set are not allowed for PxRigidBody collision.
    ///
    /// This method will set the mass, center of mass, and inertia tensor
    ///
    /// if no collision shapes are found, the inertia tensor is set to (1,1,1) and the mass to 1
    ///
    /// if massLocalPose is non-NULL, the rigid body's center of mass parameter  will be set
    /// to the user provided value (massLocalPose) and the inertia tensor will be resolved at that point.
    ///
    /// If all shapes of the actor have the same density then the overloaded method updateMassAndInertia() with a single density parameter can be used instead.
    ///
    /// Boolean. True on success else false.
bool      physx_PxRigidBodyExt_updateMassAndInertia(physx_PxRigidBody* body, float const* shapeDensities, uint32_t shapeDensityCount, physx_PxVec3 const* massLocalPose, bool includeNonSimShapes);

    /// Computation of mass properties for a rigid body actor
    ///
    /// See previous method for details.
    ///
    /// Boolean. True on success else false.
bool      physx_PxRigidBodyExt_updateMassAndInertia_1(physx_PxRigidBody* body, float density, physx_PxVec3 const* massLocalPose, bool includeNonSimShapes);

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the sum of all user-supplied
    /// shape mass values, and the inertia and center of mass are computed according to the rigid body's shapes and the per shape mass input values.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// If a single mass value should be used for the actor as a whole then the overloaded method setMassAndUpdateInertia() with a single mass parameter can be used instead.
    ///
    /// Boolean. True on success else false.
bool      physx_PxRigidBodyExt_setMassAndUpdateInertia(physx_PxRigidBody* body, float const* shapeMasses, uint32_t shapeMassCount, physx_PxVec3 const* massLocalPose, bool includeNonSimShapes);

    /// Computation of mass properties for a rigid body actor
    ///
    /// This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the user-supplied
    /// value, and the inertia and center of mass are computed according to the rigid body's shapes and the input mass.
    ///
    /// If no collision shapes are found, the inertia tensor is set to (1,1,1)
    ///
    /// Boolean. True on success else false.
bool      physx_PxRigidBodyExt_setMassAndUpdateInertia_1(physx_PxRigidBody* body, float mass, physx_PxVec3 const* massLocalPose, bool includeNonSimShapes);

    /// Compute the mass, inertia tensor and center of mass from a list of shapes.
    ///
    /// The mass properties from the combined shapes.
physx_PxMassProperties      physx_PxRigidBodyExt_computeMassPropertiesFromShapes(physx_PxShape const* const* shapes, uint32_t shapeCount);

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in global coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep,
    /// you can maintain a total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
void      physx_PxRigidBodyExt_addForceAtPos(physx_PxRigidBody* body, physx_PxVec3 const* force, physx_PxVec3 const* pos, int32_t mode, bool wakeup);

    /// Applies a force (or impulse) defined in the global coordinate frame, acting at a particular
    /// point in local coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
void      physx_PxRigidBodyExt_addForceAtLocalPos(physx_PxRigidBody* body, physx_PxVec3 const* force, physx_PxVec3 const* pos, int32_t mode, bool wakeup);

    /// Applies a force (or impulse) defined in the actor local coordinate frame, acting at a
    /// particular point in global coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
void      physx_PxRigidBodyExt_addLocalForceAtPos(physx_PxRigidBody* body, physx_PxVec3 const* force, physx_PxVec3 const* pos, int32_t mode, bool wakeup);

    /// Applies a force (or impulse) defined in the actor local coordinate frame, acting at a
    /// particular point in local coordinates, to the actor.
    ///
    /// Note that if the force does not act along the center of mass of the actor, this
    /// will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
    /// total external force on an object by calling this once every frame.
    ///
    /// if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
    /// articulation
    ///
    /// ::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the
    /// force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
    /// given point.
    ///
    /// Sleeping:
    /// This call wakes the actor if it is sleeping and the wakeup parameter is true (default).
void      physx_PxRigidBodyExt_addLocalForceAtLocalPos(physx_PxRigidBody* body, physx_PxVec3 const* force, physx_PxVec3 const* pos, int32_t mode, bool wakeup);

    /// Computes the velocity of a point given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the global frame.
physx_PxVec3      physx_PxRigidBodyExt_getVelocityAtPos(physx_PxRigidBody const* body, physx_PxVec3 const* pos);

    /// Computes the velocity of a point given in local coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point in the local frame.
physx_PxVec3      physx_PxRigidBodyExt_getLocalVelocityAtLocalPos(physx_PxRigidBody const* body, physx_PxVec3 const* pos);

    /// Computes the velocity of a point (offset from the origin of the body) given in world coordinates if it were attached to the
    /// specified body and moving with it.
    ///
    /// The velocity of point (offset from the origin of the body) in the global frame.
physx_PxVec3      physx_PxRigidBodyExt_getVelocityAtOffset(physx_PxRigidBody const* body, physx_PxVec3 const* pos);

    /// Compute the change to linear and angular velocity that would occur if an impulsive force and torque were to be applied to a specified rigid body.
    ///
    /// The rigid body is left unaffected unless a subsequent independent call is executed that actually applies the computed changes to velocity and angular velocity.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
void      physx_PxRigidBodyExt_computeVelocityDeltaFromImpulse(physx_PxRigidBody const* body, physx_PxVec3 const* impulsiveForce, physx_PxVec3 const* impulsiveTorque, physx_PxVec3* deltaLinearVelocity, physx_PxVec3* deltaAngularVelocity);

    /// Computes the linear and angular velocity change vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular velocity changes from a contact or joint when the mass/inertia ratios have been adjusted.
    ///
    /// if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.
void      physx_PxRigidBodyExt_computeVelocityDeltaFromImpulse_1(physx_PxRigidBody const* body, physx_PxTransform const* globalPose, physx_PxVec3 const* point, physx_PxVec3 const* impulse, float invMassScale, float invInertiaScale, physx_PxVec3* deltaLinearVelocity, physx_PxVec3* deltaAngularVelocity);

    /// Computes the linear and angular impulse vectors for a given impulse at a world space position taking a mass and inertia scale into account
    ///
    /// This function is useful for extracting the respective linear and angular impulses from a contact or joint when the mass/inertia ratios have been adjusted.
void      physx_PxRigidBodyExt_computeLinearAngularImpulse(physx_PxRigidBody const* body, physx_PxTransform const* globalPose, physx_PxVec3 const* point, physx_PxVec3 const* impulse, float invMassScale, float invInertiaScale, physx_PxVec3* linearImpulse, physx_PxVec3* angularImpulse);

    /// Performs a linear sweep through space with the body's geometry objects.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// If eTOUCH is returned from the filter callback, it will trigger an error and the hit will be discarded.
    ///
    /// The function sweeps all shapes attached to a given rigid body through space and reports the nearest
    /// object in the scene which intersects any of of the shapes swept paths.
    /// Information about the closest intersection is written to a [`PxSweepHit`] structure.
    ///
    /// True if a blocking hit was found.
bool      physx_PxRigidBodyExt_linearSweepSingle(physx_PxRigidBody* body, physx_PxScene* scene, physx_PxVec3 const* unitDir, float distance, uint16_t outputFlags, physx_PxSweepHit* closestHit, uint32_t* shapeIndex, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, float inflation);

    /// Performs a linear sweep through space with the body's geometry objects, returning all overlaps.
    ///
    /// Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
    ///
    /// This function sweeps all shapes attached to a given rigid body through space and reports all
    /// objects in the scene that intersect any of the shapes' swept paths until there are no more objects to report
    /// or a blocking hit is encountered.
    ///
    /// the number of touching hits. If overflow is set to true, the results are incomplete. In case of overflow there are also no guarantees that all touching hits returned are closer than the blocking hit.
uint32_t      physx_PxRigidBodyExt_linearSweepMultiple(physx_PxRigidBody* body, physx_PxScene* scene, physx_PxVec3 const* unitDir, float distance, uint16_t outputFlags, physx_PxSweepHit* touchHitBuffer, uint32_t* touchHitShapeIndices, uint32_t touchHitBufferSize, physx_PxSweepHit* block, int32_t* blockingShapeIndex, bool* overflow, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, float inflation);

    /// Retrieves the world space pose of the shape.
    ///
    /// Global pose of shape.
physx_PxTransform      physx_PxShapeExt_getGlobalPose(physx_PxShape const* shape, physx_PxRigidActor const* actor);

    /// Raycast test against the shape.
    ///
    /// Number of hits between the ray and the shape
uint32_t      physx_PxShapeExt_raycast(physx_PxShape const* shape, physx_PxRigidActor const* actor, physx_PxVec3 const* rayOrigin, physx_PxVec3 const* rayDir, float maxDist, uint16_t hitFlags, uint32_t maxHits, physx_PxRaycastHit* rayHits);

    /// Test overlap between the shape and a geometry object
    ///
    /// True if the shape overlaps the geometry object
bool      physx_PxShapeExt_overlap(physx_PxShape const* shape, physx_PxRigidActor const* actor, physx_PxGeometry const* otherGeom, physx_PxTransform const* otherGeomPose);

    /// Sweep a geometry object against the shape.
    ///
    /// Currently only box, sphere, capsule and convex mesh shapes are supported, i.e. the swept geometry object must be one of those types.
    ///
    /// True if the swept geometry object hits the shape
bool      physx_PxShapeExt_sweep(physx_PxShape const* shape, physx_PxRigidActor const* actor, physx_PxVec3 const* unitDir, float distance, physx_PxGeometry const* otherGeom, physx_PxTransform const* otherGeomPose, physx_PxSweepHit* sweepHit, uint16_t hitFlags);

    /// Retrieves the axis aligned bounding box enclosing the shape.
    ///
    /// The shape's bounding box.
physx_PxBounds3      physx_PxShapeExt_getWorldBounds(physx_PxShape const* shape, physx_PxRigidActor const* actor, float inflation);

physx_PxMeshOverlapUtil*      physx_PxMeshOverlapUtil_new_alloc();

void      physx_PxMeshOverlapUtil_delete(physx_PxMeshOverlapUtil* self_);

    /// Find the mesh triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
uint32_t      physx_PxMeshOverlapUtil_findOverlap_mut(physx_PxMeshOverlapUtil* self_, physx_PxGeometry const* geom, physx_PxTransform const* geomPose, physx_PxTriangleMeshGeometry const* meshGeom, physx_PxTransform const* meshPose);

    /// Find the height field triangles which touch the specified geometry object.
    ///
    /// Number of overlaps found. Triangle indices can then be accessed through the [`getResults`]() function.
uint32_t      physx_PxMeshOverlapUtil_findOverlap_mut_1(physx_PxMeshOverlapUtil* self_, physx_PxGeometry const* geom, physx_PxTransform const* geomPose, physx_PxHeightFieldGeometry const* hfGeom, physx_PxTransform const* hfPose);

    /// Retrieves array of triangle indices after a findOverlap call.
    ///
    /// Indices of touched triangles
uint32_t const*      physx_PxMeshOverlapUtil_getResults(physx_PxMeshOverlapUtil const* self_);

    /// Retrieves number of triangle indices after a findOverlap call.
    ///
    /// Number of touched triangles
uint32_t      physx_PxMeshOverlapUtil_getNbResults(physx_PxMeshOverlapUtil const* self_);

    /// Computes an approximate minimum translational distance (MTD) between a geometry object and a mesh.
    ///
    /// This iterative function computes an approximate vector that can be used to depenetrate a geom object
    /// from a triangle mesh. Returned depenetration vector should be applied to 'geom', to get out of the mesh.
    ///
    /// The function works best when the amount of overlap between the geom object and the mesh is small. If the
    /// geom object's center goes inside the mesh, backface culling usually kicks in, no overlap is detected,
    /// and the function does not compute an MTD vector.
    ///
    /// The function early exits if no overlap is detected after a depenetration attempt. This means that if
    /// maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
    /// been successful. Usually N = 4 gives good results.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
bool      physx_phys_PxComputeTriangleMeshPenetration(physx_PxVec3* direction, float* depth, physx_PxGeometry const* geom, physx_PxTransform const* geomPose, physx_PxTriangleMeshGeometry const* meshGeom, physx_PxTransform const* meshPose, uint32_t maxIter, uint32_t* usedIter);

    /// Computes an approximate minimum translational distance (MTD) between a geometry object and a heightfield.
    ///
    /// This iterative function computes an approximate vector that can be used to depenetrate a geom object
    /// from a heightfield. Returned depenetration vector should be applied to 'geom', to get out of the heightfield.
    ///
    /// The function works best when the amount of overlap between the geom object and the mesh is small. If the
    /// geom object's center goes inside the heightfield, backface culling usually kicks in, no overlap is detected,
    /// and the function does not compute an MTD vector.
    ///
    /// The function early exits if no overlap is detected after a depenetration attempt. This means that if
    /// maxIter = N, the code will attempt at most N iterations but it might exit earlier if depenetration has
    /// been successful. Usually N = 4 gives good results.
    ///
    /// True if the MTD has successfully been computed, i.e. if objects do overlap.
bool      physx_phys_PxComputeHeightFieldPenetration(physx_PxVec3* direction, float* depth, physx_PxGeometry const* geom, physx_PxTransform const* geomPose, physx_PxHeightFieldGeometry const* heightFieldGeom, physx_PxTransform const* heightFieldPose, uint32_t maxIter, uint32_t* usedIter);

physx_PxXmlMiscParameter      physx_PxXmlMiscParameter_new();

physx_PxXmlMiscParameter      physx_PxXmlMiscParameter_new_1(physx_PxVec3* inUpVector, physx_PxTolerancesScale inScale);

    /// Returns whether the collection is serializable with the externalReferences collection.
    ///
    /// Some definitions to explain whether a collection can be serialized or not:
    ///
    /// For definitions of
    /// requires
    /// and
    /// complete
    /// see [`PxSerialization::complete`]
    ///
    /// A serializable object is
    /// subordinate
    /// if it cannot be serialized on its own
    /// The following objects are subordinate:
    /// - articulation links
    /// - articulation joints
    /// - joints
    ///
    /// A collection C can be serialized with external references collection D iff
    /// - C is complete relative to D (no dangling references)
    /// - Every object in D required by an object in C has a valid ID (no unnamed references)
    /// - Every subordinate object in C is required by another object in C (no orphans)
    ///
    /// Whether the collection is serializable
bool      physx_PxSerialization_isSerializable(physx_PxCollection* collection, physx_PxSerializationRegistry* sr, physx_PxCollection const* externalReferences);

    /// Adds to a collection all objects such that it can be successfully serialized.
    ///
    /// A collection C is complete relative to an other collection D if every object required by C is either in C or D.
    /// This function adds objects to a collection, such that it becomes complete with respect to the exceptFor collection.
    /// Completeness is needed for serialization. See [`PxSerialization::serializeCollectionToBinary`],
    /// [`PxSerialization::serializeCollectionToXml`].
    ///
    /// Sdk objects require other sdk object according to the following rules:
    /// - joints require their actors and constraint
    /// - rigid actors require their shapes
    /// - shapes require their material(s) and mesh (triangle mesh, convex mesh or height field), if any
    /// - articulations require their links and joints
    /// - aggregates require their actors
    ///
    /// If followJoints is specified another rule is added:
    /// - actors require their joints
    ///
    /// Specifying followJoints will make whole jointed actor chains being added to the collection. Following chains
    /// is interrupted whenever a object in exceptFor is encountered.
void      physx_PxSerialization_complete(physx_PxCollection* collection, physx_PxSerializationRegistry* sr, physx_PxCollection const* exceptFor, bool followJoints);

    /// Creates PxSerialObjectId values for unnamed objects in a collection.
    ///
    /// Creates PxSerialObjectId names for unnamed objects in a collection starting at a base value and incrementing,
    /// skipping values that are already assigned to objects in the collection.
void      physx_PxSerialization_createSerialObjectIds(physx_PxCollection* collection, uint64_t base);

    /// Creates a PxCollection from XML data.
    ///
    /// a pointer to a PxCollection if successful or NULL if it failed.
physx_PxCollection*      physx_PxSerialization_createCollectionFromXml(physx_PxInputData* inputData, physx_PxCooking* cooking, physx_PxSerializationRegistry* sr, physx_PxCollection const* externalRefs, physx_PxStringTable* stringTable, physx_PxXmlMiscParameter* outArgs);

    /// Deserializes a PxCollection from memory.
    ///
    /// Creates a collection from memory. If the collection has external dependencies another collection
    /// can be provided to resolve these.
    ///
    /// The memory block provided has to be 128 bytes aligned and contain a contiguous serialized collection as written
    /// by PxSerialization::serializeCollectionToBinary. The contained binary data needs to be compatible with the current binary format version
    /// which is defined by "PX_PHYSICS_VERSION_MAJOR.PX_PHYSICS_VERSION_MINOR.PX_PHYSICS_VERSION_BUGFIX-PX_BINARY_SERIAL_VERSION".
    /// For a list of compatible sdk releases refer to the documentation of PX_BINARY_SERIAL_VERSION.
physx_PxCollection*      physx_PxSerialization_createCollectionFromBinary(void* memBlock, physx_PxSerializationRegistry* sr, physx_PxCollection const* externalRefs);

    /// Serializes a physics collection to an XML output stream.
    ///
    /// The collection to be serialized needs to be complete
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// true if the collection is successfully serialized.
bool      physx_PxSerialization_serializeCollectionToXml(physx_PxOutputStream* outputStream, physx_PxCollection* collection, physx_PxSerializationRegistry* sr, physx_PxCooking* cooking, physx_PxCollection const* externalRefs, physx_PxXmlMiscParameter* inArgs);

    /// Serializes a collection to a binary stream.
    ///
    /// Serializes a collection to a stream. In order to resolve external dependencies the externalReferences collection has to be provided.
    /// Optionally names of objects that where set for example with [`PxActor::setName`] are serialized along with the objects.
    ///
    /// The collection can be successfully serialized if isSerializable(collection) returns true. See [`isSerializable`].
    ///
    /// The implementation of the output stream needs to fulfill the requirements on the memory block input taken by
    /// PxSerialization::createCollectionFromBinary.
    ///
    /// Serialization of objects in a scene that is simultaneously being simulated is not supported and leads to undefined behavior.
    ///
    /// Whether serialization was successful
bool      physx_PxSerialization_serializeCollectionToBinary(physx_PxOutputStream* outputStream, physx_PxCollection* collection, physx_PxSerializationRegistry* sr, physx_PxCollection const* externalRefs, bool exportNames);

    /// Creates an application managed registry for serialization.
    ///
    /// PxSerializationRegistry instance.
physx_PxSerializationRegistry*      physx_PxSerialization_createSerializationRegistry(physx_PxPhysics* physics);

    /// Deletes the dispatcher.
    ///
    /// Do not keep a reference to the deleted instance.
void      physx_PxDefaultCpuDispatcher_release_mut(physx_PxDefaultCpuDispatcher* self_);

    /// Enables profiling at task level.
    ///
    /// By default enabled only in profiling builds.
void      physx_PxDefaultCpuDispatcher_setRunProfiled_mut(physx_PxDefaultCpuDispatcher* self_, bool runProfiled);

    /// Checks if profiling is enabled at task level.
    ///
    /// True if tasks should be profiled.
bool      physx_PxDefaultCpuDispatcher_getRunProfiled(physx_PxDefaultCpuDispatcher const* self_);

    /// Create default dispatcher, extensions SDK needs to be initialized first.
    ///
    /// numThreads may be zero in which case no worker thread are initialized and
    /// simulation tasks will be executed on the thread that calls PxScene::simulate()
    ///
    /// yieldProcessorCount must be greater than zero if eYIELD_PROCESSOR is the chosen mode and equal to zero for all other modes.
    ///
    /// eYIELD_THREAD and eYIELD_PROCESSOR modes will use compute resources even if the simulation is not running.
    /// It is left to users to keep threads inactive, if so desired, when no simulation is running.
physx_PxDefaultCpuDispatcher*      physx_phys_PxDefaultCpuDispatcherCreate(uint32_t numThreads, uint32_t* affinityMasks, int32_t mode, uint32_t yieldProcessorCount);

    /// Builds smooth vertex normals over a mesh.
    ///
    /// - "smooth" because smoothing groups are not supported here
    /// - takes angles into account for correct cube normals computation
    ///
    /// To use 32bit indices pass a pointer in dFaces and set wFaces to zero. Alternatively pass a pointer to
    /// wFaces and set dFaces to zero.
    ///
    /// True on success.
bool      physx_phys_PxBuildSmoothNormals(uint32_t nbTris, uint32_t nbVerts, physx_PxVec3 const* verts, uint32_t const* dFaces, uint16_t const* wFaces, physx_PxVec3* normals, bool flip);

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
physx_PxRigidDynamic*      physx_phys_PxCreateDynamic(physx_PxPhysics* sdk, physx_PxTransform const* transform, physx_PxGeometry const* geometry, physx_PxMaterial* material, float density, physx_PxTransform const* shapeOffset);

    /// simple method to create a PxRigidDynamic actor with a single PxShape.
    ///
    /// a new dynamic actor with the PxRigidBodyFlag, or NULL if it could
    /// not be constructed
physx_PxRigidDynamic*      physx_phys_PxCreateDynamic_1(physx_PxPhysics* sdk, physx_PxTransform const* transform, physx_PxShape* shape, float density);

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
physx_PxRigidDynamic*      physx_phys_PxCreateKinematic(physx_PxPhysics* sdk, physx_PxTransform const* transform, physx_PxGeometry const* geometry, physx_PxMaterial* material, float density, physx_PxTransform const* shapeOffset);

    /// simple method to create a kinematic PxRigidDynamic actor with a single PxShape.
    ///
    /// unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However,
    /// kinematics of other geometry types may not participate in simulation collision and may be used only for
    /// triggers or scene queries of moving objects under animation control. In this case the density parameter
    /// will be ignored and the created shape will be set up as a scene query only shape (see [`PxShapeFlag::eSCENE_QUERY_SHAPE`])
    ///
    /// a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could
    /// not be constructed
physx_PxRigidDynamic*      physx_phys_PxCreateKinematic_1(physx_PxPhysics* sdk, physx_PxTransform const* transform, physx_PxShape* shape, float density);

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
physx_PxRigidStatic*      physx_phys_PxCreateStatic(physx_PxPhysics* sdk, physx_PxTransform const* transform, physx_PxGeometry const* geometry, physx_PxMaterial* material, physx_PxTransform const* shapeOffset);

    /// simple method to create a PxRigidStatic actor with a single PxShape.
    ///
    /// a new static actor, or NULL if it could not be constructed
physx_PxRigidStatic*      physx_phys_PxCreateStatic_1(physx_PxPhysics* sdk, physx_PxTransform const* transform, physx_PxShape* shape);

    /// create a shape by copying attributes from another shape
    ///
    /// The function clones a PxShape. The following properties are copied:
    /// - geometry
    /// - flags
    /// - materials
    /// - actor-local pose
    /// - contact offset
    /// - rest offset
    /// - simulation filter data
    /// - query filter data
    /// - torsional patch radius
    /// - minimum torsional patch radius
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - user data
    ///
    /// the newly-created rigid static
physx_PxShape*      physx_phys_PxCloneShape(physx_PxPhysics* physicsSDK, physx_PxShape const* shape, bool isExclusive);

    /// create a static body by copying attributes from another rigid actor
    ///
    /// The function clones a PxRigidDynamic or PxRigidStatic as a PxRigidStatic. A uniform scale is applied. The following properties are copied:
    /// - shapes
    /// - actor flags
    /// - owner client and client behavior bits
    /// - dominance group
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - joints or observers
    /// - aggregate or scene membership
    /// - user data
    ///
    /// Transforms are not copied with bit-exact accuracy.
    ///
    /// the newly-created rigid static
physx_PxRigidStatic*      physx_phys_PxCloneStatic(physx_PxPhysics* physicsSDK, physx_PxTransform const* transform, physx_PxRigidActor const* actor);

    /// create a dynamic body by copying attributes from an existing body
    ///
    /// The following properties are copied:
    /// - shapes
    /// - actor flags, rigidDynamic flags and rigidDynamic lock flags
    /// - mass, moment of inertia, and center of mass frame
    /// - linear and angular velocity
    /// - linear and angular damping
    /// - maximum linear velocity
    /// - maximum angular velocity
    /// - position and velocity solver iterations
    /// - maximum depenetration velocity
    /// - sleep threshold
    /// - contact report threshold
    /// - dominance group
    /// - owner client and client behavior bits
    /// - name pointer
    /// - kinematic target
    ///
    /// The following are not copied and retain their default values:
    /// - name
    /// - joints or observers
    /// - aggregate or scene membership
    /// - sleep timer
    /// - user data
    ///
    /// Transforms are not copied with bit-exact accuracy.
    ///
    /// the newly-created rigid static
physx_PxRigidDynamic*      physx_phys_PxCloneDynamic(physx_PxPhysics* physicsSDK, physx_PxTransform const* transform, physx_PxRigidDynamic const* body);

    /// create a plane actor. The plane equation is n.x + d = 0
    ///
    /// a new static actor, or NULL if it could not be constructed
physx_PxRigidStatic*      physx_phys_PxCreatePlane(physx_PxPhysics* sdk, physx_PxPlane const* plane, physx_PxMaterial* material);

    /// scale a rigid actor by a uniform scale
    ///
    /// The geometry and relative positions of the actor are multiplied by the given scale value. If the actor is a rigid body or an
    /// articulation link and the scaleMassProps value is true, the mass properties are scaled assuming the density is constant: the
    /// center of mass is linearly scaled, the mass is multiplied by the cube of the scale, and the inertia tensor by the fifth power of the scale.
void      physx_phys_PxScaleRigidActor(physx_PxRigidActor* actor, float scale, bool scaleMassProps);

physx_PxStringTable*      physx_PxStringTableExt_createStringTable(physx_PxAllocatorCallback* inAllocator);

    /// Creates regions for PxSceneDesc, from a global box.
    ///
    /// This helper simply subdivides the given global box into a 2D grid of smaller boxes. Each one of those smaller boxes
    /// is a region of interest for the broadphase. There are nbSubdiv*nbSubdiv regions in the 2D grid. The function does not
    /// subdivide along the given up axis.
    ///
    /// This is the simplest setup one can use with PxBroadPhaseType::eMBP. A more sophisticated setup would try to cover
    /// the game world with a non-uniform set of regions (i.e. not just a grid).
    ///
    /// number of regions written out to the 'regions' array
uint32_t      physx_PxBroadPhaseExt_createRegionsFromWorldBounds(physx_PxBounds3* regions, physx_PxBounds3 const* globalBounds, uint32_t nbSubdiv, uint32_t upAxis);

    /// Raycast returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the ray.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
bool      physx_PxSceneQueryExt_raycastAny(physx_PxScene const* scene, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, float distance, physx_PxQueryHit* hit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache);

    /// Raycast returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// True if a blocking hit was found.
bool      physx_PxSceneQueryExt_raycastSingle(physx_PxScene const* scene, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, float distance, uint16_t outputFlags, physx_PxRaycastHit* hit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache);

    /// Raycast returning multiple results.
    ///
    /// Find all rigid actors that get hit along the ray. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
int32_t      physx_PxSceneQueryExt_raycastMultiple(physx_PxScene const* scene, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, float distance, uint16_t outputFlags, physx_PxRaycastHit* hitBuffer, uint32_t hitBufferSize, bool* blockingHit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache);

    /// Sweep returning any blocking hit, not necessarily the closest.
    ///
    /// Returns whether any rigid actor is hit along the sweep path.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
bool      physx_PxSceneQueryExt_sweepAny(physx_PxScene const* scene, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxVec3 const* unitDir, float distance, uint16_t queryFlags, physx_PxQueryHit* hit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, float inflation);

    /// Sweep returning a single result.
    ///
    /// Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// True if a blocking hit was found.
bool      physx_PxSceneQueryExt_sweepSingle(physx_PxScene const* scene, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxVec3 const* unitDir, float distance, uint16_t outputFlags, physx_PxSweepHit* hit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, float inflation);

    /// Sweep returning multiple results.
    ///
    /// Find all rigid actors that get hit along the sweep. Each result contains data as specified by the outputFlags field.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
int32_t      physx_PxSceneQueryExt_sweepMultiple(physx_PxScene const* scene, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxVec3 const* unitDir, float distance, uint16_t outputFlags, physx_PxSweepHit* hitBuffer, uint32_t hitBufferSize, bool* blockingHit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall, physx_PxQueryCache const* cache, float inflation);

    /// Test overlap between a geometry and objects in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both get written to the hit buffer.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// Number of hits in the buffer, or -1 if the buffer overflowed.
int32_t      physx_PxSceneQueryExt_overlapMultiple(physx_PxScene const* scene, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxOverlapHit* hitBuffer, uint32_t hitBufferSize, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall);

    /// Test returning, for a given geometry, any overlapping object in the scene.
    ///
    /// Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both trigger a hit.
    ///
    /// PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
    ///
    /// True if an overlap was found.
bool      physx_PxSceneQueryExt_overlapAny(physx_PxScene const* scene, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxOverlapHit* hit, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall);

void      physx_PxBatchQueryExt_release_mut(physx_PxBatchQueryExt* self_);

    /// Performs a raycast against objects in the scene.
    ///
    /// Touching hits are not ordered.
    ///
    /// Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxRaycastBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
physx_PxRaycastBuffer*      physx_PxBatchQueryExt_raycast_mut(physx_PxBatchQueryExt* self_, physx_PxVec3 const* origin, physx_PxVec3 const* unitDir, float distance, uint16_t maxNbTouches, uint16_t hitFlags, physx_PxQueryFilterData const* filterData, physx_PxQueryCache const* cache);

    /// Performs a sweep test against objects in the scene.
    ///
    /// Touching hits are not ordered.
    ///
    /// If a shape from the scene is already overlapping with the query shape in its starting position,
    /// the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxSweepBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
physx_PxSweepBuffer*      physx_PxBatchQueryExt_sweep_mut(physx_PxBatchQueryExt* self_, physx_PxGeometry const* geometry, physx_PxTransform const* pose, physx_PxVec3 const* unitDir, float distance, uint16_t maxNbTouches, uint16_t hitFlags, physx_PxQueryFilterData const* filterData, physx_PxQueryCache const* cache, float inflation);

    /// Performs an overlap test of a given geometry against objects in the scene.
    ///
    /// Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see [`PxQueryHitType`]).
    ///
    /// eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
    ///
    /// If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
    ///
    /// This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
    /// and overlapping writes from different threads may result in undefined behavior).
    ///
    /// Returns a PxOverlapBuffer pointer that will store the result of the query after execute() is completed.
    /// This will point either to an element of the buffer allocated on construction or to a user buffer passed to the constructor.
physx_PxOverlapBuffer*      physx_PxBatchQueryExt_overlap_mut(physx_PxBatchQueryExt* self_, physx_PxGeometry const* geometry, physx_PxTransform const* pose, uint16_t maxNbTouches, physx_PxQueryFilterData const* filterData, physx_PxQueryCache const* cache);

void      physx_PxBatchQueryExt_execute_mut(physx_PxBatchQueryExt* self_);

    /// Create a PxBatchQueryExt without the need for pre-allocated result or touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
physx_PxBatchQueryExt*      physx_phys_PxCreateBatchQueryExt(physx_PxScene const* scene, physx_PxQueryFilterCallback* queryFilterCallback, uint32_t maxNbRaycasts, uint32_t maxNbRaycastTouches, uint32_t maxNbSweeps, uint32_t maxNbSweepTouches, uint32_t maxNbOverlaps, uint32_t maxNbOverlapTouches);

    /// Create a PxBatchQueryExt with user-supplied result and touch buffers.
    ///
    /// Returns a PxBatchQueryExt instance. A NULL pointer will be returned if the subsequent allocations fail or if any of the arguments are illegal.
    /// In the event that a NULL pointer is returned a corresponding error will be issued to the error stream.
physx_PxBatchQueryExt*      physx_phys_PxCreateBatchQueryExt_1(physx_PxScene const* scene, physx_PxQueryFilterCallback* queryFilterCallback, physx_PxRaycastBuffer* raycastBuffers, uint32_t maxNbRaycasts, physx_PxRaycastHit* raycastTouches, uint32_t maxNbRaycastTouches, physx_PxSweepBuffer* sweepBuffers, uint32_t maxNbSweeps, physx_PxSweepHit* sweepTouches, uint32_t maxNbSweepTouches, physx_PxOverlapBuffer* overlapBuffers, uint32_t maxNbOverlaps, physx_PxOverlapHit* overlapTouches, uint32_t maxNbOverlapTouches);

    /// Creates an external scene query system.
    ///
    /// An external SQ system is the part of a PxScene that deals with scene queries (SQ). This is usually taken care of
    /// by an internal implementation inside PxScene, but it is also possible to re-route all SQ calls to an external
    /// implementation, potentially opening the door to some customizations in behavior and features for advanced users.
    ///
    /// The following external SQ system is an example of how an implementation would look like. It re-uses much of the
    /// same code as the internal version, but it could be re-implemented in a completely different way to match users'
    /// specific needs.
    ///
    /// An external SQ system instance
physx_PxSceneQuerySystem*      physx_phys_PxCreateExternalSceneQuerySystem(physx_PxSceneQueryDesc const* desc, uint64_t contextID);

    /// Adds a pruner to the system.
    ///
    /// The internal PhysX scene-query system uses two regular pruners (one for static shapes, one for dynamic shapes) and an optional
    /// compound pruner. Our custom scene query system supports an arbitrary number of regular pruners.
    ///
    /// This can be useful to reduce the load on each pruner, in particular during updates, when internal trees are rebuilt in the
    /// background. On the other hand this implementation simply iterates over all created pruners to perform queries, so their cost
    /// might increase if a large number of pruners is used.
    ///
    /// In any case this serves as an example of how the PxSceneQuerySystem API can be used to customize scene queries.
    ///
    /// A pruner index
uint32_t      physx_PxCustomSceneQuerySystem_addPruner_mut(physx_PxCustomSceneQuerySystem* self_, int32_t primaryType, int32_t secondaryType, uint32_t preallocated);

    /// Start custom build-steps for all pruners
    ///
    /// This function is used in combination with customBuildstep() and finishCustomBuildstep() to let users take control
    /// of the pruners' build-step
    /// &
    /// commit calls - basically the pruners' update functions. These functions should be used
    /// with the PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED update mode, otherwise the build-steps will happen
    /// automatically in fetchResults. For N pruners it can be more efficient to use these custom build-step functions to
    /// perform the updates in parallel:
    ///
    /// - call startCustomBuildstep() first (one synchronous call)
    /// - for each pruner, call customBuildstep() (asynchronous calls from multiple threads)
    /// - once it is done, call finishCustomBuildstep() to finish the update (synchronous call)
    ///
    /// The multi-threaded update is more efficient here than what it is in PxScene, because the "flushShapes()" call is
    /// also multi-threaded (while it is not in PxScene).
    ///
    /// Note that users are responsible for locks here, and these calls should not overlap with other SQ calls. In particular
    /// one should not add new objects to the SQ system or perform queries while these calls are happening.
    ///
    /// The number of pruners in the system.
uint32_t      physx_PxCustomSceneQuerySystem_startCustomBuildstep_mut(physx_PxCustomSceneQuerySystem* self_);

    /// Perform a custom build-step for a given pruner.
void      physx_PxCustomSceneQuerySystem_customBuildstep_mut(physx_PxCustomSceneQuerySystem* self_, uint32_t index);

    /// Finish custom build-steps
    ///
    /// Call this function once after all the customBuildstep() calls are done.
void      physx_PxCustomSceneQuerySystem_finishCustomBuildstep_mut(physx_PxCustomSceneQuerySystem* self_);

void      physx_PxCustomSceneQuerySystemAdapter_delete(physx_PxCustomSceneQuerySystemAdapter* self_);

    /// Gets a pruner index for an actor/shape.
    ///
    /// This user-defined function tells the system in which pruner a given actor/shape should go.
    ///
    /// The returned index must be valid, i.e. it must have been previously returned to users by PxCustomSceneQuerySystem::addPruner.
    ///
    /// A pruner index for this actor/shape.
uint32_t      physx_PxCustomSceneQuerySystemAdapter_getPrunerIndex(physx_PxCustomSceneQuerySystemAdapter const* self_, physx_PxRigidActor const* actor, physx_PxShape const* shape);

    /// Pruner filtering callback.
    ///
    /// This will be called for each query to validate whether it should process a given pruner.
    ///
    /// True to process the pruner, false to skip it entirely
bool      physx_PxCustomSceneQuerySystemAdapter_processPruner(physx_PxCustomSceneQuerySystemAdapter const* self_, uint32_t prunerIndex, physx_PxQueryThreadContext const* context, physx_PxQueryFilterData const* filterData, physx_PxQueryFilterCallback* filterCall);

    /// Creates a custom scene query system.
    ///
    /// This is similar to PxCreateExternalSceneQuerySystem, except this function creates a PxCustomSceneQuerySystem object.
    /// It can be plugged to PxScene the same way, via PxSceneDesc::sceneQuerySystem.
    ///
    /// A custom SQ system instance
physx_PxCustomSceneQuerySystem*      physx_phys_PxCreateCustomSceneQuerySystem(int32_t sceneQueryUpdateMode, uint64_t contextID, physx_PxCustomSceneQuerySystemAdapter const* adapter, bool usesTreeOfPruners);

    /// Computes closest polygon of the convex hull geometry for a given impact point
    /// and impact direction. When doing sweeps against a scene, one might want to delay
    /// the rather expensive computation of the hit face index for convexes until it is clear
    /// the information is really needed and then use this method to get the corresponding
    /// face index.
    ///
    /// Closest face index of the convex geometry.
uint32_t      physx_phys_PxFindFaceIndex(physx_PxConvexMeshGeometry const* convexGeom, physx_PxTransform const* geomPose, physx_PxVec3 const* impactPos, physx_PxVec3 const* unitDir);

    /// Sets the sampling radius
    ///
    /// Returns true if the sampling was successful and false if there was a problem. Usually an internal overflow is the problem for very big meshes or very small sampling radii.
bool      physx_PxPoissonSampler_setSamplingRadius_mut(physx_PxPoissonSampler* self_, float samplingRadius);

    /// Adds new Poisson Samples inside the sphere specified
void      physx_PxPoissonSampler_addSamplesInSphere_mut(physx_PxPoissonSampler* self_, physx_PxVec3 const* sphereCenter, float sphereRadius, bool createVolumeSamples);

    /// Adds new Poisson Samples inside the box specified
void      physx_PxPoissonSampler_addSamplesInBox_mut(physx_PxPoissonSampler* self_, physx_PxBounds3 const* axisAlignedBox, physx_PxQuat const* boxOrientation, bool createVolumeSamples);

void      physx_PxPoissonSampler_delete(physx_PxPoissonSampler* self_);

    /// Creates a shape sampler
    ///
    /// Returns the sampler
physx_PxPoissonSampler*      physx_phys_PxCreateShapeSampler(physx_PxGeometry const* geometry, physx_PxTransform const* transform, physx_PxBounds3 const* worldBounds, float initialSamplingRadius, int32_t numSampleAttemptsAroundPoint);

    /// Checks whether a point is inside the triangle mesh
    ///
    /// Returns true if the point is inside the triangle mesh
bool      physx_PxTriangleMeshPoissonSampler_isPointInTriangleMesh_mut(physx_PxTriangleMeshPoissonSampler* self_, physx_PxVec3 const* p);

void      physx_PxTriangleMeshPoissonSampler_delete(physx_PxTriangleMeshPoissonSampler* self_);

    /// Creates a triangle mesh sampler
    ///
    /// Returns the sampler
physx_PxTriangleMeshPoissonSampler*      physx_phys_PxCreateTriangleMeshSampler(uint32_t const* triangles, uint32_t numTriangles, physx_PxVec3 const* vertices, uint32_t numVertices, float initialSamplingRadius, int32_t numSampleAttemptsAroundPoint);

    /// Returns the index of the tetrahedron that contains a point
    ///
    /// The index of the tetrahedon containing the point, -1 if not tetrahedron contains the opoint
int32_t      physx_PxTetrahedronMeshExt_findTetrahedronContainingPoint(physx_PxTetrahedronMesh const* mesh, physx_PxVec3 const* point, physx_PxVec4* bary, float tolerance);

    /// Returns the index of the tetrahedron closest to a point
    ///
    /// The index of the tetrahedon closest to the point
int32_t      physx_PxTetrahedronMeshExt_findTetrahedronClosestToPoint(physx_PxTetrahedronMesh const* mesh, physx_PxVec3 const* point, physx_PxVec4* bary);

    /// Initialize the PhysXExtensions library.
    ///
    /// This should be called before calling any functions or methods in extensions which may require allocation.
    ///
    /// This function does not need to be called before creating a PxDefaultAllocator object.
bool      physx_phys_PxInitExtensions(physx_PxPhysics* physics, physx_PxPvd* pvd);

    /// Shut down the PhysXExtensions library.
    ///
    /// This function should be called to cleanly shut down the PhysXExtensions library before application exit.
    ///
    /// This function is required to be called to release foundation usage.
void      physx_phys_PxCloseExtensions();

physx_PxRepXObject      physx_PxRepXObject_new(char const* inTypeName, void const* inSerializable, uint64_t inId);

bool      physx_PxRepXObject_isValid(physx_PxRepXObject const* self_);

physx_PxRepXInstantiationArgs      physx_PxRepXInstantiationArgs_new(physx_PxPhysics* inPhysics, physx_PxCooking* inCooking, physx_PxStringTable* inStringTable);

    /// The type this Serializer is meant to operate on.
char const*      physx_PxRepXSerializer_getTypeName_mut(physx_PxRepXSerializer* self_);

    /// Convert from a RepX object to a key-value pair hierarchy
void      physx_PxRepXSerializer_objectToFile_mut(physx_PxRepXSerializer* self_, physx_PxRepXObject const* inLiveObject, physx_PxCollection* inCollection, physx_XmlWriter* inWriter, physx_MemoryBuffer* inTempBuffer, physx_PxRepXInstantiationArgs* inArgs);

    /// Convert from a descriptor to a live object.  Must be an object of this Serializer type.
    ///
    /// The new live object.  It can be an invalid object if the instantiation cannot take place.
physx_PxRepXObject      physx_PxRepXSerializer_fileToObject_mut(physx_PxRepXSerializer* self_, physx_XmlReader* inReader, physx_XmlMemoryAllocator* inAllocator, physx_PxRepXInstantiationArgs* inArgs, physx_PxCollection* inCollection);

    /// Connects the SDK to the PhysX Visual Debugger application.
bool      physx_PxPvd_connect_mut(physx_PxPvd* self_, physx_PxPvdTransport* transport, uint8_t flags);

    /// Disconnects the SDK from the PhysX Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
void      physx_PxPvd_disconnect_mut(physx_PxPvd* self_);

    /// Return if connection to PVD is created.
bool      physx_PxPvd_isConnected_mut(physx_PxPvd* self_, bool useCachedStatus);

    /// returns the PVD data transport
    /// returns NULL if no transport is present.
physx_PxPvdTransport*      physx_PxPvd_getTransport_mut(physx_PxPvd* self_);

    /// Retrieves the PVD flags. See PxPvdInstrumentationFlags.
uint8_t      physx_PxPvd_getInstrumentationFlags_mut(physx_PxPvd* self_);

    /// Releases the pvd instance.
void      physx_PxPvd_release_mut(physx_PxPvd* self_);

    /// Create a pvd instance.
physx_PxPvd*      physx_phys_PxCreatePvd(physx_PxFoundation* foundation);

    /// Connects to the Visual Debugger application.
    /// return True if success
bool      physx_PxPvdTransport_connect_mut(physx_PxPvdTransport* self_);

    /// Disconnects from the Visual Debugger application.
    /// If we are still connected, this will kill the entire debugger connection.
void      physx_PxPvdTransport_disconnect_mut(physx_PxPvdTransport* self_);

    /// Return if connection to PVD is created.
bool      physx_PxPvdTransport_isConnected_mut(physx_PxPvdTransport* self_);

    /// write bytes to the other endpoint of the connection. should lock before witre. If an error occurs
    /// this connection will assume to be dead.
bool      physx_PxPvdTransport_write_mut(physx_PxPvdTransport* self_, uint8_t const* inBytes, uint32_t inLength);

physx_PxPvdTransport*      physx_PxPvdTransport_lock_mut(physx_PxPvdTransport* self_);

void      physx_PxPvdTransport_unlock_mut(physx_PxPvdTransport* self_);

    /// send any data and block until we know it is at least on the wire.
void      physx_PxPvdTransport_flush_mut(physx_PxPvdTransport* self_);

    /// Return size of written data.
uint64_t      physx_PxPvdTransport_getWrittenDataSize_mut(physx_PxPvdTransport* self_);

void      physx_PxPvdTransport_release_mut(physx_PxPvdTransport* self_);

    /// Create a default socket transport.
physx_PxPvdTransport*      physx_phys_PxDefaultPvdSocketTransportCreate(char const* host, int32_t port, uint32_t timeoutInMilliseconds);

    /// Create a default file transport.
physx_PxPvdTransport*      physx_phys_PxDefaultPvdFileTransportCreate(char const* name);
