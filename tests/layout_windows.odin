package tests
import  "core:testing"
import physx ".."

@(test)
test_layout_AllocatorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(AllocatorCallback) == 8, "Wrong size for type AllocatorCallback, expected 8 got %v", size_of(AllocatorCallback))
}

@(test)
test_layout_AssertHandler :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(AssertHandler) == 8, "Wrong size for type AssertHandler, expected 8 got %v", size_of(AssertHandler))
}

@(test)
test_layout_Foundation :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Foundation) == 8, "Wrong size for type Foundation, expected 8 got %v", size_of(Foundation))
}



@(test)
test_layout_VirtualAllocatorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(VirtualAllocatorCallback) == 8, "Wrong size for type VirtualAllocatorCallback, expected 8 got %v", size_of(VirtualAllocatorCallback))
}

@(test)
test_layout_VirtualAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(VirtualAllocator) == 16, "Wrong size for type VirtualAllocator, expected 16 got %v", size_of(VirtualAllocator))
}




@(test)
test_layout_BitAndByte :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BitAndByte) == 1, "Wrong size for type BitAndByte, expected 1 got %v", size_of(BitAndByte))
}

@(test)
test_layout_BitMap :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BitMap) == 16, "Wrong size for type BitMap, expected 16 got %v", size_of(BitMap))
}

@(test)
test_layout_Vec3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Vec3, y) == 4, "Wrong offset for Vec3.y, expected 4 got %v", offset_of(Vec3, y))
    testing.expectf(t, offset_of(Vec3, z) == 8, "Wrong offset for Vec3.z, expected 8 got %v", offset_of(Vec3, z))
    testing.expectf(t, size_of(Vec3) == 12, "Wrong size for type Vec3, expected 12 got %v", size_of(Vec3))
}

@(test)
test_layout_Vec3Padded :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Vec3Padded, padding) == 12, "Wrong offset for Vec3Padded.padding, expected 12 got %v", offset_of(Vec3Padded, padding))
    testing.expectf(t, size_of(Vec3Padded) == 16, "Wrong size for type Vec3Padded, expected 16 got %v", size_of(Vec3Padded))
}

@(test)
test_layout_Quat :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Quat, y) == 4, "Wrong offset for Quat.y, expected 4 got %v", offset_of(Quat, y))
    testing.expectf(t, offset_of(Quat, z) == 8, "Wrong offset for Quat.z, expected 8 got %v", offset_of(Quat, z))
    testing.expectf(t, offset_of(Quat, w) == 12, "Wrong offset for Quat.w, expected 12 got %v", offset_of(Quat, w))
    testing.expectf(t, size_of(Quat) == 16, "Wrong size for type Quat, expected 16 got %v", size_of(Quat))
}

@(test)
test_layout_Transform :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Transform, p) == 16, "Wrong offset for Transform.p, expected 16 got %v", offset_of(Transform, p))
    testing.expectf(t, size_of(Transform) == 28, "Wrong size for type Transform, expected 28 got %v", size_of(Transform))
}

@(test)
test_layout_TransformPadded :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TransformPadded, padding) == 28, "Wrong offset for TransformPadded.padding, expected 28 got %v", offset_of(TransformPadded, padding))
    testing.expectf(t, size_of(TransformPadded) == 32, "Wrong size for type TransformPadded, expected 32 got %v", size_of(TransformPadded))
}

@(test)
test_layout_Mat33 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Mat33, column1) == 12, "Wrong offset for Mat33.column1, expected 12 got %v", offset_of(Mat33, column1))
    testing.expectf(t, offset_of(Mat33, column2) == 24, "Wrong offset for Mat33.column2, expected 24 got %v", offset_of(Mat33, column2))
    testing.expectf(t, size_of(Mat33) == 36, "Wrong size for type Mat33, expected 36 got %v", size_of(Mat33))
}

@(test)
test_layout_Bounds3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Bounds3, maximum) == 12, "Wrong offset for Bounds3.maximum, expected 12 got %v", offset_of(Bounds3, maximum))
    testing.expectf(t, size_of(Bounds3) == 24, "Wrong size for type Bounds3, expected 24 got %v", size_of(Bounds3))
}

@(test)
test_layout_ErrorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ErrorCallback) == 8, "Wrong size for type ErrorCallback, expected 8 got %v", size_of(ErrorCallback))
}

@(test)
test_layout_AllocationListener :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(AllocationListener) == 8, "Wrong size for type AllocationListener, expected 8 got %v", size_of(AllocationListener))
}

@(test)
test_layout_BroadcastingAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BroadcastingAllocator) == 176, "Wrong size for type BroadcastingAllocator, expected 176 got %v", size_of(BroadcastingAllocator))
}

@(test)
test_layout_BroadcastingErrorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BroadcastingErrorCallback) == 160, "Wrong size for type BroadcastingErrorCallback, expected 160 got %v", size_of(BroadcastingErrorCallback))
}

@(test)
test_layout_InputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(InputStream) == 8, "Wrong size for type InputStream, expected 8 got %v", size_of(InputStream))
}

@(test)
test_layout_InputData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(InputData) == 8, "Wrong size for type InputData, expected 8 got %v", size_of(InputData))
}

@(test)
test_layout_OutputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(OutputStream) == 8, "Wrong size for type OutputStream, expected 8 got %v", size_of(OutputStream))
}

@(test)
test_layout_Vec4 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Vec4, y) == 4, "Wrong offset for Vec4.y, expected 4 got %v", offset_of(Vec4, y))
    testing.expectf(t, offset_of(Vec4, z) == 8, "Wrong offset for Vec4.z, expected 8 got %v", offset_of(Vec4, z))
    testing.expectf(t, offset_of(Vec4, w) == 12, "Wrong offset for Vec4.w, expected 12 got %v", offset_of(Vec4, w))
    testing.expectf(t, size_of(Vec4) == 16, "Wrong size for type Vec4, expected 16 got %v", size_of(Vec4))
}

@(test)
test_layout_Mat44 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Mat44, column1) == 16, "Wrong offset for Mat44.column1, expected 16 got %v", offset_of(Mat44, column1))
    testing.expectf(t, offset_of(Mat44, column2) == 32, "Wrong offset for Mat44.column2, expected 32 got %v", offset_of(Mat44, column2))
    testing.expectf(t, offset_of(Mat44, column3) == 48, "Wrong offset for Mat44.column3, expected 48 got %v", offset_of(Mat44, column3))
    testing.expectf(t, size_of(Mat44) == 64, "Wrong size for type Mat44, expected 64 got %v", size_of(Mat44))
}

@(test)
test_layout_Plane :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Plane, d) == 12, "Wrong offset for Plane.d, expected 12 got %v", offset_of(Plane, d))
    testing.expectf(t, size_of(Plane) == 16, "Wrong size for type Plane, expected 16 got %v", size_of(Plane))
}



@(test)
test_layout_ReadWriteLock :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ReadWriteLock) == 8, "Wrong size for type ReadWriteLock, expected 8 got %v", size_of(ReadWriteLock))
}

@(test)
test_layout_ProfilerCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ProfilerCallback) == 8, "Wrong size for type ProfilerCallback, expected 8 got %v", size_of(ProfilerCallback))
}

@(test)
test_layout_ProfileScoped :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ProfileScoped, mEventName) == 8, "Wrong offset for ProfileScoped.mEventName, expected 8 got %v", offset_of(ProfileScoped, mEventName))
    testing.expectf(t, offset_of(ProfileScoped, mProfilerData) == 16, "Wrong offset for ProfileScoped.mProfilerData, expected 16 got %v", offset_of(ProfileScoped, mProfilerData))
    testing.expectf(t, offset_of(ProfileScoped, mContextId) == 24, "Wrong offset for ProfileScoped.mContextId, expected 24 got %v", offset_of(ProfileScoped, mContextId))
    testing.expectf(t, offset_of(ProfileScoped, mDetached) == 32, "Wrong offset for ProfileScoped.mDetached, expected 32 got %v", offset_of(ProfileScoped, mDetached))
    testing.expectf(t, size_of(ProfileScoped) == 40, "Wrong size for type ProfileScoped, expected 40 got %v", size_of(ProfileScoped))
}

@(test)
test_layout_SListEntry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SListEntry) == 16, "Wrong size for type SListEntry, expected 16 got %v", size_of(SListEntry))
}



@(test)
test_layout_Runnable :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Runnable) == 8, "Wrong size for type Runnable, expected 8 got %v", size_of(Runnable))
}

@(test)
test_layout_CounterFrequencyToTensOfNanos :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(CounterFrequencyToTensOfNanos, mDenominator) == 8, "Wrong offset for CounterFrequencyToTensOfNanos.mDenominator, expected 8 got %v", offset_of(CounterFrequencyToTensOfNanos, mDenominator))
    testing.expectf(t, size_of(CounterFrequencyToTensOfNanos) == 16, "Wrong size for type CounterFrequencyToTensOfNanos, expected 16 got %v", size_of(CounterFrequencyToTensOfNanos))
}

@(test)
test_layout_Time :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Time) == 8, "Wrong size for type Time, expected 8 got %v", size_of(Time))
}

@(test)
test_layout_Vec2 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Vec2, y) == 4, "Wrong offset for Vec2.y, expected 4 got %v", offset_of(Vec2, y))
    testing.expectf(t, size_of(Vec2) == 8, "Wrong size for type Vec2, expected 8 got %v", size_of(Vec2))
}

@(test)
test_layout_StridedData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(StridedData, data) == 8, "Wrong offset for StridedData.data, expected 8 got %v", offset_of(StridedData, data))
    testing.expectf(t, size_of(StridedData) == 16, "Wrong size for type StridedData, expected 16 got %v", size_of(StridedData))
}

@(test)
test_layout_BoundedData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BoundedData, count) == 16, "Wrong offset for BoundedData.count, expected 16 got %v", offset_of(BoundedData, count))
    testing.expectf(t, size_of(BoundedData) == 24, "Wrong size for type BoundedData, expected 24 got %v", size_of(BoundedData))
}

@(test)
test_layout_DebugPoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(DebugPoint, color) == 12, "Wrong offset for DebugPoint.color, expected 12 got %v", offset_of(DebugPoint, color))
    testing.expectf(t, size_of(DebugPoint) == 16, "Wrong size for type DebugPoint, expected 16 got %v", size_of(DebugPoint))
}

@(test)
test_layout_DebugLine :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(DebugLine, color0) == 12, "Wrong offset for DebugLine.color0, expected 12 got %v", offset_of(DebugLine, color0))
    testing.expectf(t, offset_of(DebugLine, pos1) == 16, "Wrong offset for DebugLine.pos1, expected 16 got %v", offset_of(DebugLine, pos1))
    testing.expectf(t, offset_of(DebugLine, color1) == 28, "Wrong offset for DebugLine.color1, expected 28 got %v", offset_of(DebugLine, color1))
    testing.expectf(t, size_of(DebugLine) == 32, "Wrong size for type DebugLine, expected 32 got %v", size_of(DebugLine))
}

@(test)
test_layout_DebugTriangle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(DebugTriangle, color0) == 12, "Wrong offset for DebugTriangle.color0, expected 12 got %v", offset_of(DebugTriangle, color0))
    testing.expectf(t, offset_of(DebugTriangle, pos1) == 16, "Wrong offset for DebugTriangle.pos1, expected 16 got %v", offset_of(DebugTriangle, pos1))
    testing.expectf(t, offset_of(DebugTriangle, color1) == 28, "Wrong offset for DebugTriangle.color1, expected 28 got %v", offset_of(DebugTriangle, color1))
    testing.expectf(t, offset_of(DebugTriangle, pos2) == 32, "Wrong offset for DebugTriangle.pos2, expected 32 got %v", offset_of(DebugTriangle, pos2))
    testing.expectf(t, offset_of(DebugTriangle, color2) == 44, "Wrong offset for DebugTriangle.color2, expected 44 got %v", offset_of(DebugTriangle, color2))
    testing.expectf(t, size_of(DebugTriangle) == 48, "Wrong size for type DebugTriangle, expected 48 got %v", size_of(DebugTriangle))
}

@(test)
test_layout_DebugText :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(DebugText, size) == 12, "Wrong offset for DebugText.size, expected 12 got %v", offset_of(DebugText, size))
    testing.expectf(t, offset_of(DebugText, color) == 16, "Wrong offset for DebugText.color, expected 16 got %v", offset_of(DebugText, color))
    testing.expectf(t, offset_of(DebugText, string) == 24, "Wrong offset for DebugText.string, expected 24 got %v", offset_of(DebugText, string))
    testing.expectf(t, size_of(DebugText) == 32, "Wrong size for type DebugText, expected 32 got %v", size_of(DebugText))
}

@(test)
test_layout_RenderBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RenderBuffer) == 8, "Wrong size for type RenderBuffer, expected 8 got %v", size_of(RenderBuffer))
}

@(test)
test_layout_ProcessPxBaseCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ProcessPxBaseCallback) == 8, "Wrong size for type ProcessPxBaseCallback, expected 8 got %v", size_of(ProcessPxBaseCallback))
}

@(test)
test_layout_SerializationContext :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SerializationContext) == 8, "Wrong size for type SerializationContext, expected 8 got %v", size_of(SerializationContext))
}

@(test)
test_layout_DeserializationContext :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DeserializationContext) == 16, "Wrong size for type DeserializationContext, expected 16 got %v", size_of(DeserializationContext))
}

@(test)
test_layout_SerializationRegistry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SerializationRegistry) == 8, "Wrong size for type SerializationRegistry, expected 8 got %v", size_of(SerializationRegistry))
}

@(test)
test_layout_Collection :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Collection) == 8, "Wrong size for type Collection, expected 8 got %v", size_of(Collection))
}

@(test)
test_layout_Base :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Base) == 16, "Wrong size for type Base, expected 16 got %v", size_of(Base))
}

@(test)
test_layout_RefCounted :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RefCounted) == 16, "Wrong size for type RefCounted, expected 16 got %v", size_of(RefCounted))
}

@(test)
test_layout_TolerancesScale :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TolerancesScale, speed) == 4, "Wrong offset for TolerancesScale.speed, expected 4 got %v", offset_of(TolerancesScale, speed))
    testing.expectf(t, size_of(TolerancesScale) == 8, "Wrong size for type TolerancesScale, expected 8 got %v", size_of(TolerancesScale))
}

@(test)
test_layout_StringTable :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(StringTable) == 8, "Wrong size for type StringTable, expected 8 got %v", size_of(StringTable))
}

@(test)
test_layout_Serializer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Serializer) == 8, "Wrong size for type Serializer, expected 8 got %v", size_of(Serializer))
}

@(test)
test_layout_MetaDataEntry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(MetaDataEntry, name) == 8, "Wrong offset for MetaDataEntry.name, expected 8 got %v", offset_of(MetaDataEntry, name))
    testing.expectf(t, offset_of(MetaDataEntry, offset) == 16, "Wrong offset for MetaDataEntry.offset, expected 16 got %v", offset_of(MetaDataEntry, offset))
    testing.expectf(t, offset_of(MetaDataEntry, size) == 20, "Wrong offset for MetaDataEntry.size, expected 20 got %v", offset_of(MetaDataEntry, size))
    testing.expectf(t, offset_of(MetaDataEntry, count) == 24, "Wrong offset for MetaDataEntry.count, expected 24 got %v", offset_of(MetaDataEntry, count))
    testing.expectf(t, offset_of(MetaDataEntry, offsetSize) == 28, "Wrong offset for MetaDataEntry.offsetSize, expected 28 got %v", offset_of(MetaDataEntry, offsetSize))
    testing.expectf(t, offset_of(MetaDataEntry, flags) == 32, "Wrong offset for MetaDataEntry.flags, expected 32 got %v", offset_of(MetaDataEntry, flags))
    testing.expectf(t, offset_of(MetaDataEntry, alignment) == 36, "Wrong offset for MetaDataEntry.alignment, expected 36 got %v", offset_of(MetaDataEntry, alignment))
    testing.expectf(t, size_of(MetaDataEntry) == 40, "Wrong size for type MetaDataEntry, expected 40 got %v", size_of(MetaDataEntry))
}

@(test)
test_layout_InsertionCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(InsertionCallback) == 8, "Wrong size for type InsertionCallback, expected 8 got %v", size_of(InsertionCallback))
}

@(test)
test_layout_TaskManager :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(TaskManager) == 8, "Wrong size for type TaskManager, expected 8 got %v", size_of(TaskManager))
}

@(test)
test_layout_CpuDispatcher :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CpuDispatcher) == 8, "Wrong size for type CpuDispatcher, expected 8 got %v", size_of(CpuDispatcher))
}

@(test)
test_layout_BaseTask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BaseTask) == 24, "Wrong size for type BaseTask, expected 24 got %v", size_of(BaseTask))
}

@(test)
test_layout_Task :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Task) == 32, "Wrong size for type Task, expected 32 got %v", size_of(Task))
}

@(test)
test_layout_LightCpuTask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(LightCpuTask) == 40, "Wrong size for type LightCpuTask, expected 40 got %v", size_of(LightCpuTask))
}

@(test)
test_layout_Geometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Geometry, mTypePadding) == 4, "Wrong offset for Geometry.mTypePadding, expected 4 got %v", offset_of(Geometry, mTypePadding))
    testing.expectf(t, size_of(Geometry) == 8, "Wrong size for type Geometry, expected 8 got %v", size_of(Geometry))
}

@(test)
test_layout_BoxGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BoxGeometry, halfExtents) == 8, "Wrong offset for BoxGeometry.halfExtents, expected 8 got %v", offset_of(BoxGeometry, halfExtents))
    testing.expectf(t, size_of(BoxGeometry) == 20, "Wrong size for type BoxGeometry, expected 20 got %v", size_of(BoxGeometry))
}

@(test)
test_layout_BVHRaycastCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BVHRaycastCallback) == 8, "Wrong size for type BVHRaycastCallback, expected 8 got %v", size_of(BVHRaycastCallback))
}

@(test)
test_layout_BVHOverlapCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BVHOverlapCallback) == 8, "Wrong size for type BVHOverlapCallback, expected 8 got %v", size_of(BVHOverlapCallback))
}

@(test)
test_layout_BVHTraversalCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BVHTraversalCallback) == 8, "Wrong size for type BVHTraversalCallback, expected 8 got %v", size_of(BVHTraversalCallback))
}

@(test)
test_layout_BVH :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BVH) == 16, "Wrong size for type BVH, expected 16 got %v", size_of(BVH))
}

@(test)
test_layout_CapsuleGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(CapsuleGeometry, radius) == 8, "Wrong offset for CapsuleGeometry.radius, expected 8 got %v", offset_of(CapsuleGeometry, radius))
    testing.expectf(t, offset_of(CapsuleGeometry, halfHeight) == 12, "Wrong offset for CapsuleGeometry.halfHeight, expected 12 got %v", offset_of(CapsuleGeometry, halfHeight))
    testing.expectf(t, size_of(CapsuleGeometry) == 16, "Wrong size for type CapsuleGeometry, expected 16 got %v", size_of(CapsuleGeometry))
}

@(test)
test_layout_HullPolygon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(HullPolygon, mNbVerts) == 16, "Wrong offset for HullPolygon.mNbVerts, expected 16 got %v", offset_of(HullPolygon, mNbVerts))
    testing.expectf(t, offset_of(HullPolygon, mIndexBase) == 18, "Wrong offset for HullPolygon.mIndexBase, expected 18 got %v", offset_of(HullPolygon, mIndexBase))
    testing.expectf(t, size_of(HullPolygon) == 20, "Wrong size for type HullPolygon, expected 20 got %v", size_of(HullPolygon))
}

@(test)
test_layout_ConvexMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ConvexMesh) == 16, "Wrong size for type ConvexMesh, expected 16 got %v", size_of(ConvexMesh))
}

@(test)
test_layout_MeshScale :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(MeshScale, rotation) == 12, "Wrong offset for MeshScale.rotation, expected 12 got %v", offset_of(MeshScale, rotation))
    testing.expectf(t, size_of(MeshScale) == 28, "Wrong size for type MeshScale, expected 28 got %v", size_of(MeshScale))
}

@(test)
test_layout_ConvexMeshGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConvexMeshGeometry, scale) == 8, "Wrong offset for ConvexMeshGeometry.scale, expected 8 got %v", offset_of(ConvexMeshGeometry, scale))
    testing.expectf(t, offset_of(ConvexMeshGeometry, convexMesh) == 40, "Wrong offset for ConvexMeshGeometry.convexMesh, expected 40 got %v", offset_of(ConvexMeshGeometry, convexMesh))
    testing.expectf(t, offset_of(ConvexMeshGeometry, meshFlags) == 48, "Wrong offset for ConvexMeshGeometry.meshFlags, expected 48 got %v", offset_of(ConvexMeshGeometry, meshFlags))
    testing.expectf(t, size_of(ConvexMeshGeometry) == 56, "Wrong size for type ConvexMeshGeometry, expected 56 got %v", size_of(ConvexMeshGeometry))
}

@(test)
test_layout_SphereGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SphereGeometry, radius) == 8, "Wrong offset for SphereGeometry.radius, expected 8 got %v", offset_of(SphereGeometry, radius))
    testing.expectf(t, size_of(SphereGeometry) == 12, "Wrong size for type SphereGeometry, expected 12 got %v", size_of(SphereGeometry))
}

@(test)
test_layout_PlaneGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PlaneGeometry) == 8, "Wrong size for type PlaneGeometry, expected 8 got %v", size_of(PlaneGeometry))
}

@(test)
test_layout_TriangleMeshGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TriangleMeshGeometry, scale) == 8, "Wrong offset for TriangleMeshGeometry.scale, expected 8 got %v", offset_of(TriangleMeshGeometry, scale))
    testing.expectf(t, offset_of(TriangleMeshGeometry, meshFlags) == 36, "Wrong offset for TriangleMeshGeometry.meshFlags, expected 36 got %v", offset_of(TriangleMeshGeometry, meshFlags))
    testing.expectf(t, offset_of(TriangleMeshGeometry, triangleMesh) == 40, "Wrong offset for TriangleMeshGeometry.triangleMesh, expected 40 got %v", offset_of(TriangleMeshGeometry, triangleMesh))
    testing.expectf(t, size_of(TriangleMeshGeometry) == 48, "Wrong size for type TriangleMeshGeometry, expected 48 got %v", size_of(TriangleMeshGeometry))
}

@(test)
test_layout_HeightFieldGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(HeightFieldGeometry, heightField) == 8, "Wrong offset for HeightFieldGeometry.heightField, expected 8 got %v", offset_of(HeightFieldGeometry, heightField))
    testing.expectf(t, offset_of(HeightFieldGeometry, heightScale) == 16, "Wrong offset for HeightFieldGeometry.heightScale, expected 16 got %v", offset_of(HeightFieldGeometry, heightScale))
    testing.expectf(t, offset_of(HeightFieldGeometry, rowScale) == 20, "Wrong offset for HeightFieldGeometry.rowScale, expected 20 got %v", offset_of(HeightFieldGeometry, rowScale))
    testing.expectf(t, offset_of(HeightFieldGeometry, columnScale) == 24, "Wrong offset for HeightFieldGeometry.columnScale, expected 24 got %v", offset_of(HeightFieldGeometry, columnScale))
    testing.expectf(t, offset_of(HeightFieldGeometry, heightFieldFlags) == 28, "Wrong offset for HeightFieldGeometry.heightFieldFlags, expected 28 got %v", offset_of(HeightFieldGeometry, heightFieldFlags))
    testing.expectf(t, size_of(HeightFieldGeometry) == 32, "Wrong size for type HeightFieldGeometry, expected 32 got %v", size_of(HeightFieldGeometry))
}

@(test)
test_layout_ParticleSystemGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ParticleSystemGeometry, mSolverType) == 8, "Wrong offset for ParticleSystemGeometry.mSolverType, expected 8 got %v", offset_of(ParticleSystemGeometry, mSolverType))
    testing.expectf(t, size_of(ParticleSystemGeometry) == 12, "Wrong size for type ParticleSystemGeometry, expected 12 got %v", size_of(ParticleSystemGeometry))
}

@(test)
test_layout_HairSystemGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(HairSystemGeometry) == 8, "Wrong size for type HairSystemGeometry, expected 8 got %v", size_of(HairSystemGeometry))
}

@(test)
test_layout_TetrahedronMeshGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TetrahedronMeshGeometry, tetrahedronMesh) == 8, "Wrong offset for TetrahedronMeshGeometry.tetrahedronMesh, expected 8 got %v", offset_of(TetrahedronMeshGeometry, tetrahedronMesh))
    testing.expectf(t, size_of(TetrahedronMeshGeometry) == 16, "Wrong size for type TetrahedronMeshGeometry, expected 16 got %v", size_of(TetrahedronMeshGeometry))
}

@(test)
test_layout_QueryHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(QueryHit) == 4, "Wrong size for type QueryHit, expected 4 got %v", size_of(QueryHit))
}

@(test)
test_layout_LocationHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(LocationHit, flags) == 4, "Wrong offset for LocationHit.flags, expected 4 got %v", offset_of(LocationHit, flags))
    testing.expectf(t, offset_of(LocationHit, position) == 8, "Wrong offset for LocationHit.position, expected 8 got %v", offset_of(LocationHit, position))
    testing.expectf(t, offset_of(LocationHit, normal) == 20, "Wrong offset for LocationHit.normal, expected 20 got %v", offset_of(LocationHit, normal))
    testing.expectf(t, offset_of(LocationHit, distance) == 32, "Wrong offset for LocationHit.distance, expected 32 got %v", offset_of(LocationHit, distance))
    testing.expectf(t, size_of(LocationHit) == 36, "Wrong size for type LocationHit, expected 36 got %v", size_of(LocationHit))
}

@(test)
test_layout_GeomRaycastHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GeomRaycastHit, u) == 36, "Wrong offset for GeomRaycastHit.u, expected 36 got %v", offset_of(GeomRaycastHit, u))
    testing.expectf(t, offset_of(GeomRaycastHit, v) == 40, "Wrong offset for GeomRaycastHit.v, expected 40 got %v", offset_of(GeomRaycastHit, v))
    testing.expectf(t, size_of(GeomRaycastHit) == 44, "Wrong size for type GeomRaycastHit, expected 44 got %v", size_of(GeomRaycastHit))
}

@(test)
test_layout_GeomOverlapHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(GeomOverlapHit) == 4, "Wrong size for type GeomOverlapHit, expected 4 got %v", size_of(GeomOverlapHit))
}

@(test)
test_layout_GeomSweepHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(GeomSweepHit) == 36, "Wrong size for type GeomSweepHit, expected 36 got %v", size_of(GeomSweepHit))
}

@(test)
test_layout_GeomIndexPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GeomIndexPair, id1) == 4, "Wrong offset for GeomIndexPair.id1, expected 4 got %v", offset_of(GeomIndexPair, id1))
    testing.expectf(t, size_of(GeomIndexPair) == 8, "Wrong size for type GeomIndexPair, expected 8 got %v", size_of(GeomIndexPair))
}


@(test)
test_layout_CustomGeometryType :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CustomGeometryType) == 4, "Wrong size for type CustomGeometryType, expected 4 got %v", size_of(CustomGeometryType))
}

@(test)
test_layout_CustomGeometryCallbacks :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CustomGeometryCallbacks) == 8, "Wrong size for type CustomGeometryCallbacks, expected 8 got %v", size_of(CustomGeometryCallbacks))
}

@(test)
test_layout_CustomGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(CustomGeometry, callbacks) == 8, "Wrong offset for CustomGeometry.callbacks, expected 8 got %v", offset_of(CustomGeometry, callbacks))
    testing.expectf(t, size_of(CustomGeometry) == 16, "Wrong size for type CustomGeometry, expected 16 got %v", size_of(CustomGeometry))
}

@(test)
test_layout_GeometryHolder :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(GeometryHolder) == 56, "Wrong size for type GeometryHolder, expected 56 got %v", size_of(GeometryHolder))
}


@(test)
test_layout_HeightFieldSample :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(HeightFieldSample, materialIndex0) == 2, "Wrong offset for HeightFieldSample.materialIndex0, expected 2 got %v", offset_of(HeightFieldSample, materialIndex0))
    testing.expectf(t, offset_of(HeightFieldSample, materialIndex1) == 3, "Wrong offset for HeightFieldSample.materialIndex1, expected 3 got %v", offset_of(HeightFieldSample, materialIndex1))
    testing.expectf(t, size_of(HeightFieldSample) == 4, "Wrong size for type HeightFieldSample, expected 4 got %v", size_of(HeightFieldSample))
}

@(test)
test_layout_HeightField :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(HeightField) == 16, "Wrong size for type HeightField, expected 16 got %v", size_of(HeightField))
}

@(test)
test_layout_HeightFieldDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(HeightFieldDesc, nbColumns) == 4, "Wrong offset for HeightFieldDesc.nbColumns, expected 4 got %v", offset_of(HeightFieldDesc, nbColumns))
    testing.expectf(t, offset_of(HeightFieldDesc, format) == 8, "Wrong offset for HeightFieldDesc.format, expected 8 got %v", offset_of(HeightFieldDesc, format))
    testing.expectf(t, offset_of(HeightFieldDesc, samples) == 16, "Wrong offset for HeightFieldDesc.samples, expected 16 got %v", offset_of(HeightFieldDesc, samples))
    testing.expectf(t, offset_of(HeightFieldDesc, convexEdgeThreshold) == 32, "Wrong offset for HeightFieldDesc.convexEdgeThreshold, expected 32 got %v", offset_of(HeightFieldDesc, convexEdgeThreshold))
    testing.expectf(t, offset_of(HeightFieldDesc, flags) == 36, "Wrong offset for HeightFieldDesc.flags, expected 36 got %v", offset_of(HeightFieldDesc, flags))
    testing.expectf(t, size_of(HeightFieldDesc) == 40, "Wrong size for type HeightFieldDesc, expected 40 got %v", size_of(HeightFieldDesc))
}


@(test)
test_layout_SimpleTriangleMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SimpleTriangleMesh, triangles) == 24, "Wrong offset for SimpleTriangleMesh.triangles, expected 24 got %v", offset_of(SimpleTriangleMesh, triangles))
    testing.expectf(t, offset_of(SimpleTriangleMesh, flags) == 48, "Wrong offset for SimpleTriangleMesh.flags, expected 48 got %v", offset_of(SimpleTriangleMesh, flags))
    testing.expectf(t, size_of(SimpleTriangleMesh) == 56, "Wrong size for type SimpleTriangleMesh, expected 56 got %v", size_of(SimpleTriangleMesh))
}

@(test)
test_layout_Triangle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Triangle) == 36, "Wrong size for type Triangle, expected 36 got %v", size_of(Triangle))
}

@(test)
test_layout_TrianglePadded :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TrianglePadded, padding) == 36, "Wrong offset for TrianglePadded.padding, expected 36 got %v", offset_of(TrianglePadded, padding))
    testing.expectf(t, size_of(TrianglePadded) == 40, "Wrong size for type TrianglePadded, expected 40 got %v", size_of(TrianglePadded))
}

@(test)
test_layout_TriangleMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(TriangleMesh) == 16, "Wrong size for type TriangleMesh, expected 16 got %v", size_of(TriangleMesh))
}

@(test)
test_layout_BVH34TriangleMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BVH34TriangleMesh) == 16, "Wrong size for type BVH34TriangleMesh, expected 16 got %v", size_of(BVH34TriangleMesh))
}

@(test)
test_layout_Tetrahedron :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Tetrahedron) == 48, "Wrong size for type Tetrahedron, expected 48 got %v", size_of(Tetrahedron))
}

@(test)
test_layout_SoftBodyAuxData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SoftBodyAuxData) == 16, "Wrong size for type SoftBodyAuxData, expected 16 got %v", size_of(SoftBodyAuxData))
}

@(test)
test_layout_TetrahedronMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(TetrahedronMesh) == 16, "Wrong size for type TetrahedronMesh, expected 16 got %v", size_of(TetrahedronMesh))
}

@(test)
test_layout_SoftBodyMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SoftBodyMesh) == 16, "Wrong size for type SoftBodyMesh, expected 16 got %v", size_of(SoftBodyMesh))
}

@(test)
test_layout_CollisionMeshMappingData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CollisionMeshMappingData) == 8, "Wrong size for type CollisionMeshMappingData, expected 8 got %v", size_of(CollisionMeshMappingData))
}




@(test)
test_layout_CollisionTetrahedronMeshData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CollisionTetrahedronMeshData) == 8, "Wrong size for type CollisionTetrahedronMeshData, expected 8 got %v", size_of(CollisionTetrahedronMeshData))
}

@(test)
test_layout_SimulationTetrahedronMeshData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SimulationTetrahedronMeshData) == 8, "Wrong size for type SimulationTetrahedronMeshData, expected 8 got %v", size_of(SimulationTetrahedronMeshData))
}

@(test)
test_layout_Actor :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Actor, userData) == 16, "Wrong offset for Actor.userData, expected 16 got %v", offset_of(Actor, userData))
    testing.expectf(t, size_of(Actor) == 24, "Wrong size for type Actor, expected 24 got %v", size_of(Actor))
}

@(test)
test_layout_Aggregate :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Aggregate, userData) == 16, "Wrong offset for Aggregate.userData, expected 16 got %v", offset_of(Aggregate, userData))
    testing.expectf(t, size_of(Aggregate) == 24, "Wrong size for type Aggregate, expected 24 got %v", size_of(Aggregate))
}

@(test)
test_layout_SpringModifiers :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SpringModifiers, damping) == 4, "Wrong offset for SpringModifiers.damping, expected 4 got %v", offset_of(SpringModifiers, damping))
    testing.expectf(t, size_of(SpringModifiers) == 16, "Wrong size for type SpringModifiers, expected 16 got %v", size_of(SpringModifiers))
}

@(test)
test_layout_RestitutionModifiers :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(RestitutionModifiers, velocityThreshold) == 4, "Wrong offset for RestitutionModifiers.velocityThreshold, expected 4 got %v", offset_of(RestitutionModifiers, velocityThreshold))
    testing.expectf(t, size_of(RestitutionModifiers) == 16, "Wrong size for type RestitutionModifiers, expected 16 got %v", size_of(RestitutionModifiers))
}


@(test)
test_layout_OneDConstraint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(OneDConstraint, geometricError) == 12, "Wrong offset for OneDConstraint.geometricError, expected 12 got %v", offset_of(OneDConstraint, geometricError))
    testing.expectf(t, offset_of(OneDConstraint, angular0) == 16, "Wrong offset for OneDConstraint.angular0, expected 16 got %v", offset_of(OneDConstraint, angular0))
    testing.expectf(t, offset_of(OneDConstraint, velocityTarget) == 28, "Wrong offset for OneDConstraint.velocityTarget, expected 28 got %v", offset_of(OneDConstraint, velocityTarget))
    testing.expectf(t, offset_of(OneDConstraint, linear1) == 32, "Wrong offset for OneDConstraint.linear1, expected 32 got %v", offset_of(OneDConstraint, linear1))
    testing.expectf(t, offset_of(OneDConstraint, minImpulse) == 44, "Wrong offset for OneDConstraint.minImpulse, expected 44 got %v", offset_of(OneDConstraint, minImpulse))
    testing.expectf(t, offset_of(OneDConstraint, angular1) == 48, "Wrong offset for OneDConstraint.angular1, expected 48 got %v", offset_of(OneDConstraint, angular1))
    testing.expectf(t, offset_of(OneDConstraint, maxImpulse) == 60, "Wrong offset for OneDConstraint.maxImpulse, expected 60 got %v", offset_of(OneDConstraint, maxImpulse))
    testing.expectf(t, offset_of(OneDConstraint, mods) == 64, "Wrong offset for OneDConstraint.mods, expected 64 got %v", offset_of(OneDConstraint, mods))
    testing.expectf(t, offset_of(OneDConstraint, forInternalUse) == 80, "Wrong offset for OneDConstraint.forInternalUse, expected 80 got %v", offset_of(OneDConstraint, forInternalUse))
    testing.expectf(t, offset_of(OneDConstraint, flags) == 84, "Wrong offset for OneDConstraint.flags, expected 84 got %v", offset_of(OneDConstraint, flags))
    testing.expectf(t, offset_of(OneDConstraint, solveHint) == 86, "Wrong offset for OneDConstraint.solveHint, expected 86 got %v", offset_of(OneDConstraint, solveHint))
    testing.expectf(t, size_of(OneDConstraint) == 96, "Wrong size for type OneDConstraint, expected 96 got %v", size_of(OneDConstraint))
}

@(test)
test_layout_ConstraintInvMassScale :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConstraintInvMassScale, angular0) == 4, "Wrong offset for ConstraintInvMassScale.angular0, expected 4 got %v", offset_of(ConstraintInvMassScale, angular0))
    testing.expectf(t, offset_of(ConstraintInvMassScale, linear1) == 8, "Wrong offset for ConstraintInvMassScale.linear1, expected 8 got %v", offset_of(ConstraintInvMassScale, linear1))
    testing.expectf(t, offset_of(ConstraintInvMassScale, angular1) == 12, "Wrong offset for ConstraintInvMassScale.angular1, expected 12 got %v", offset_of(ConstraintInvMassScale, angular1))
    testing.expectf(t, size_of(ConstraintInvMassScale) == 16, "Wrong size for type ConstraintInvMassScale, expected 16 got %v", size_of(ConstraintInvMassScale))
}

@(test)
test_layout_ConstraintVisualizer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ConstraintVisualizer) == 8, "Wrong size for type ConstraintVisualizer, expected 8 got %v", size_of(ConstraintVisualizer))
}

@(test)
test_layout_ConstraintConnector :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ConstraintConnector) == 8, "Wrong size for type ConstraintConnector, expected 8 got %v", size_of(ConstraintConnector))
}

@(test)
test_layout_ContactPoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPoint, separation) == 12, "Wrong offset for ContactPoint.separation, expected 12 got %v", offset_of(ContactPoint, separation))
    testing.expectf(t, offset_of(ContactPoint, point) == 16, "Wrong offset for ContactPoint.point, expected 16 got %v", offset_of(ContactPoint, point))
    testing.expectf(t, offset_of(ContactPoint, maxImpulse) == 28, "Wrong offset for ContactPoint.maxImpulse, expected 28 got %v", offset_of(ContactPoint, maxImpulse))
    testing.expectf(t, offset_of(ContactPoint, targetVel) == 32, "Wrong offset for ContactPoint.targetVel, expected 32 got %v", offset_of(ContactPoint, targetVel))
    testing.expectf(t, offset_of(ContactPoint, staticFriction) == 44, "Wrong offset for ContactPoint.staticFriction, expected 44 got %v", offset_of(ContactPoint, staticFriction))
    testing.expectf(t, offset_of(ContactPoint, materialFlags) == 48, "Wrong offset for ContactPoint.materialFlags, expected 48 got %v", offset_of(ContactPoint, materialFlags))
    testing.expectf(t, offset_of(ContactPoint, internalFaceIndex1) == 52, "Wrong offset for ContactPoint.internalFaceIndex1, expected 52 got %v", offset_of(ContactPoint, internalFaceIndex1))
    testing.expectf(t, offset_of(ContactPoint, dynamicFriction) == 56, "Wrong offset for ContactPoint.dynamicFriction, expected 56 got %v", offset_of(ContactPoint, dynamicFriction))
    testing.expectf(t, offset_of(ContactPoint, restitution) == 60, "Wrong offset for ContactPoint.restitution, expected 60 got %v", offset_of(ContactPoint, restitution))
    testing.expectf(t, offset_of(ContactPoint, damping) == 64, "Wrong offset for ContactPoint.damping, expected 64 got %v", offset_of(ContactPoint, damping))
    testing.expectf(t, size_of(ContactPoint) == 80, "Wrong size for type ContactPoint, expected 80 got %v", size_of(ContactPoint))
}

@(test)
test_layout_SolverBody :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SolverBody, maxSolverNormalProgress) == 12, "Wrong offset for SolverBody.maxSolverNormalProgress, expected 12 got %v", offset_of(SolverBody, maxSolverNormalProgress))
    testing.expectf(t, offset_of(SolverBody, maxSolverFrictionProgress) == 14, "Wrong offset for SolverBody.maxSolverFrictionProgress, expected 14 got %v", offset_of(SolverBody, maxSolverFrictionProgress))
    testing.expectf(t, offset_of(SolverBody, angularState) == 16, "Wrong offset for SolverBody.angularState, expected 16 got %v", offset_of(SolverBody, angularState))
    testing.expectf(t, offset_of(SolverBody, solverProgress) == 28, "Wrong offset for SolverBody.solverProgress, expected 28 got %v", offset_of(SolverBody, solverProgress))
    testing.expectf(t, size_of(SolverBody) == 32, "Wrong size for type SolverBody, expected 32 got %v", size_of(SolverBody))
}

@(test)
test_layout_SolverBodyData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SolverBodyData, invMass) == 12, "Wrong offset for SolverBodyData.invMass, expected 12 got %v", offset_of(SolverBodyData, invMass))
    testing.expectf(t, offset_of(SolverBodyData, angularVelocity) == 16, "Wrong offset for SolverBodyData.angularVelocity, expected 16 got %v", offset_of(SolverBodyData, angularVelocity))
    testing.expectf(t, offset_of(SolverBodyData, reportThreshold) == 28, "Wrong offset for SolverBodyData.reportThreshold, expected 28 got %v", offset_of(SolverBodyData, reportThreshold))
    testing.expectf(t, offset_of(SolverBodyData, sqrtInvInertia) == 32, "Wrong offset for SolverBodyData.sqrtInvInertia, expected 32 got %v", offset_of(SolverBodyData, sqrtInvInertia))
    testing.expectf(t, offset_of(SolverBodyData, penBiasClamp) == 68, "Wrong offset for SolverBodyData.penBiasClamp, expected 68 got %v", offset_of(SolverBodyData, penBiasClamp))
    testing.expectf(t, offset_of(SolverBodyData, nodeIndex) == 72, "Wrong offset for SolverBodyData.nodeIndex, expected 72 got %v", offset_of(SolverBodyData, nodeIndex))
    testing.expectf(t, offset_of(SolverBodyData, maxContactImpulse) == 76, "Wrong offset for SolverBodyData.maxContactImpulse, expected 76 got %v", offset_of(SolverBodyData, maxContactImpulse))
    testing.expectf(t, offset_of(SolverBodyData, body2World) == 80, "Wrong offset for SolverBodyData.body2World, expected 80 got %v", offset_of(SolverBodyData, body2World))
    testing.expectf(t, offset_of(SolverBodyData, pad) == 108, "Wrong offset for SolverBodyData.pad, expected 108 got %v", offset_of(SolverBodyData, pad))
    testing.expectf(t, size_of(SolverBodyData) == 112, "Wrong size for type SolverBodyData, expected 112 got %v", size_of(SolverBodyData))
}

@(test)
test_layout_ConstraintBatchHeader :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConstraintBatchHeader, stride) == 4, "Wrong offset for ConstraintBatchHeader.stride, expected 4 got %v", offset_of(ConstraintBatchHeader, stride))
    testing.expectf(t, offset_of(ConstraintBatchHeader, constraintType) == 6, "Wrong offset for ConstraintBatchHeader.constraintType, expected 6 got %v", offset_of(ConstraintBatchHeader, constraintType))
    testing.expectf(t, size_of(ConstraintBatchHeader) == 8, "Wrong size for type ConstraintBatchHeader, expected 8 got %v", size_of(ConstraintBatchHeader))
}

@(test)
test_layout_SolverConstraintDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SolverConstraintDesc, bodyB) == 8, "Wrong offset for SolverConstraintDesc.bodyB, expected 8 got %v", offset_of(SolverConstraintDesc, bodyB))
    testing.expectf(t, offset_of(SolverConstraintDesc, bodyADataIndex) == 16, "Wrong offset for SolverConstraintDesc.bodyADataIndex, expected 16 got %v", offset_of(SolverConstraintDesc, bodyADataIndex))
    testing.expectf(t, offset_of(SolverConstraintDesc, bodyBDataIndex) == 20, "Wrong offset for SolverConstraintDesc.bodyBDataIndex, expected 20 got %v", offset_of(SolverConstraintDesc, bodyBDataIndex))
    testing.expectf(t, offset_of(SolverConstraintDesc, linkIndexA) == 24, "Wrong offset for SolverConstraintDesc.linkIndexA, expected 24 got %v", offset_of(SolverConstraintDesc, linkIndexA))
    testing.expectf(t, offset_of(SolverConstraintDesc, linkIndexB) == 28, "Wrong offset for SolverConstraintDesc.linkIndexB, expected 28 got %v", offset_of(SolverConstraintDesc, linkIndexB))
    testing.expectf(t, offset_of(SolverConstraintDesc, constraint) == 32, "Wrong offset for SolverConstraintDesc.constraint, expected 32 got %v", offset_of(SolverConstraintDesc, constraint))
    testing.expectf(t, offset_of(SolverConstraintDesc, writeBack) == 40, "Wrong offset for SolverConstraintDesc.writeBack, expected 40 got %v", offset_of(SolverConstraintDesc, writeBack))
    testing.expectf(t, offset_of(SolverConstraintDesc, progressA) == 48, "Wrong offset for SolverConstraintDesc.progressA, expected 48 got %v", offset_of(SolverConstraintDesc, progressA))
    testing.expectf(t, offset_of(SolverConstraintDesc, progressB) == 50, "Wrong offset for SolverConstraintDesc.progressB, expected 50 got %v", offset_of(SolverConstraintDesc, progressB))
    testing.expectf(t, offset_of(SolverConstraintDesc, constraintLengthOver16) == 52, "Wrong offset for SolverConstraintDesc.constraintLengthOver16, expected 52 got %v", offset_of(SolverConstraintDesc, constraintLengthOver16))
    testing.expectf(t, size_of(SolverConstraintDesc) == 64, "Wrong size for type SolverConstraintDesc, expected 64 got %v", size_of(SolverConstraintDesc))
}

@(test)
test_layout_SolverConstraintPrepDescBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, desc) == 16, "Wrong offset for SolverConstraintPrepDescBase.desc, expected 16 got %v", offset_of(SolverConstraintPrepDescBase, desc))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, body0) == 24, "Wrong offset for SolverConstraintPrepDescBase.body0, expected 24 got %v", offset_of(SolverConstraintPrepDescBase, body0))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, body1) == 32, "Wrong offset for SolverConstraintPrepDescBase.body1, expected 32 got %v", offset_of(SolverConstraintPrepDescBase, body1))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, data0) == 40, "Wrong offset for SolverConstraintPrepDescBase.data0, expected 40 got %v", offset_of(SolverConstraintPrepDescBase, data0))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, data1) == 48, "Wrong offset for SolverConstraintPrepDescBase.data1, expected 48 got %v", offset_of(SolverConstraintPrepDescBase, data1))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, bodyFrame0) == 56, "Wrong offset for SolverConstraintPrepDescBase.bodyFrame0, expected 56 got %v", offset_of(SolverConstraintPrepDescBase, bodyFrame0))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, bodyFrame1) == 84, "Wrong offset for SolverConstraintPrepDescBase.bodyFrame1, expected 84 got %v", offset_of(SolverConstraintPrepDescBase, bodyFrame1))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, bodyState0) == 112, "Wrong offset for SolverConstraintPrepDescBase.bodyState0, expected 112 got %v", offset_of(SolverConstraintPrepDescBase, bodyState0))
    testing.expectf(t, offset_of(SolverConstraintPrepDescBase, bodyState1) == 116, "Wrong offset for SolverConstraintPrepDescBase.bodyState1, expected 116 got %v", offset_of(SolverConstraintPrepDescBase, bodyState1))
    testing.expectf(t, size_of(SolverConstraintPrepDescBase) == 128, "Wrong size for type SolverConstraintPrepDescBase, expected 128 got %v", size_of(SolverConstraintPrepDescBase))
}

@(test)
test_layout_SolverConstraintPrepDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, rows) == 128, "Wrong offset for SolverConstraintPrepDesc.rows, expected 128 got %v", offset_of(SolverConstraintPrepDesc, rows))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, numRows) == 136, "Wrong offset for SolverConstraintPrepDesc.numRows, expected 136 got %v", offset_of(SolverConstraintPrepDesc, numRows))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, linBreakForce) == 140, "Wrong offset for SolverConstraintPrepDesc.linBreakForce, expected 140 got %v", offset_of(SolverConstraintPrepDesc, linBreakForce))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, angBreakForce) == 144, "Wrong offset for SolverConstraintPrepDesc.angBreakForce, expected 144 got %v", offset_of(SolverConstraintPrepDesc, angBreakForce))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, minResponseThreshold) == 148, "Wrong offset for SolverConstraintPrepDesc.minResponseThreshold, expected 148 got %v", offset_of(SolverConstraintPrepDesc, minResponseThreshold))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, writeback) == 152, "Wrong offset for SolverConstraintPrepDesc.writeback, expected 152 got %v", offset_of(SolverConstraintPrepDesc, writeback))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, disablePreprocessing) == 160, "Wrong offset for SolverConstraintPrepDesc.disablePreprocessing, expected 160 got %v", offset_of(SolverConstraintPrepDesc, disablePreprocessing))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, improvedSlerp) == 161, "Wrong offset for SolverConstraintPrepDesc.improvedSlerp, expected 161 got %v", offset_of(SolverConstraintPrepDesc, improvedSlerp))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, driveLimitsAreForces) == 162, "Wrong offset for SolverConstraintPrepDesc.driveLimitsAreForces, expected 162 got %v", offset_of(SolverConstraintPrepDesc, driveLimitsAreForces))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, extendedLimits) == 163, "Wrong offset for SolverConstraintPrepDesc.extendedLimits, expected 163 got %v", offset_of(SolverConstraintPrepDesc, extendedLimits))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, disableConstraint) == 164, "Wrong offset for SolverConstraintPrepDesc.disableConstraint, expected 164 got %v", offset_of(SolverConstraintPrepDesc, disableConstraint))
    testing.expectf(t, offset_of(SolverConstraintPrepDesc, body0WorldOffset) == 168, "Wrong offset for SolverConstraintPrepDesc.body0WorldOffset, expected 168 got %v", offset_of(SolverConstraintPrepDesc, body0WorldOffset))
    testing.expectf(t, size_of(SolverConstraintPrepDesc) == 192, "Wrong size for type SolverConstraintPrepDesc, expected 192 got %v", size_of(SolverConstraintPrepDesc))
}

@(test)
test_layout_SolverContactDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SolverContactDesc, desc) == 16, "Wrong offset for SolverContactDesc.desc, expected 16 got %v", offset_of(SolverContactDesc, desc))
    testing.expectf(t, offset_of(SolverContactDesc, body0) == 24, "Wrong offset for SolverContactDesc.body0, expected 24 got %v", offset_of(SolverContactDesc, body0))
    testing.expectf(t, offset_of(SolverContactDesc, body1) == 32, "Wrong offset for SolverContactDesc.body1, expected 32 got %v", offset_of(SolverContactDesc, body1))
    testing.expectf(t, offset_of(SolverContactDesc, data0) == 40, "Wrong offset for SolverContactDesc.data0, expected 40 got %v", offset_of(SolverContactDesc, data0))
    testing.expectf(t, offset_of(SolverContactDesc, data1) == 48, "Wrong offset for SolverContactDesc.data1, expected 48 got %v", offset_of(SolverContactDesc, data1))
    testing.expectf(t, offset_of(SolverContactDesc, bodyFrame0) == 56, "Wrong offset for SolverContactDesc.bodyFrame0, expected 56 got %v", offset_of(SolverContactDesc, bodyFrame0))
    testing.expectf(t, offset_of(SolverContactDesc, bodyFrame1) == 84, "Wrong offset for SolverContactDesc.bodyFrame1, expected 84 got %v", offset_of(SolverContactDesc, bodyFrame1))
    testing.expectf(t, offset_of(SolverContactDesc, bodyState0) == 112, "Wrong offset for SolverContactDesc.bodyState0, expected 112 got %v", offset_of(SolverContactDesc, bodyState0))
    testing.expectf(t, offset_of(SolverContactDesc, bodyState1) == 116, "Wrong offset for SolverContactDesc.bodyState1, expected 116 got %v", offset_of(SolverContactDesc, bodyState1))
    testing.expectf(t, offset_of(SolverContactDesc, shapeInteraction) == 128, "Wrong offset for SolverContactDesc.shapeInteraction, expected 128 got %v", offset_of(SolverContactDesc, shapeInteraction))
    testing.expectf(t, offset_of(SolverContactDesc, contacts) == 136, "Wrong offset for SolverContactDesc.contacts, expected 136 got %v", offset_of(SolverContactDesc, contacts))
    testing.expectf(t, offset_of(SolverContactDesc, numContacts) == 144, "Wrong offset for SolverContactDesc.numContacts, expected 144 got %v", offset_of(SolverContactDesc, numContacts))
    testing.expectf(t, offset_of(SolverContactDesc, hasMaxImpulse) == 148, "Wrong offset for SolverContactDesc.hasMaxImpulse, expected 148 got %v", offset_of(SolverContactDesc, hasMaxImpulse))
    testing.expectf(t, offset_of(SolverContactDesc, disableStrongFriction) == 149, "Wrong offset for SolverContactDesc.disableStrongFriction, expected 149 got %v", offset_of(SolverContactDesc, disableStrongFriction))
    testing.expectf(t, offset_of(SolverContactDesc, hasForceThresholds) == 150, "Wrong offset for SolverContactDesc.hasForceThresholds, expected 150 got %v", offset_of(SolverContactDesc, hasForceThresholds))
    testing.expectf(t, offset_of(SolverContactDesc, restDistance) == 152, "Wrong offset for SolverContactDesc.restDistance, expected 152 got %v", offset_of(SolverContactDesc, restDistance))
    testing.expectf(t, offset_of(SolverContactDesc, maxCCDSeparation) == 156, "Wrong offset for SolverContactDesc.maxCCDSeparation, expected 156 got %v", offset_of(SolverContactDesc, maxCCDSeparation))
    testing.expectf(t, offset_of(SolverContactDesc, frictionPtr) == 160, "Wrong offset for SolverContactDesc.frictionPtr, expected 160 got %v", offset_of(SolverContactDesc, frictionPtr))
    testing.expectf(t, offset_of(SolverContactDesc, frictionCount) == 168, "Wrong offset for SolverContactDesc.frictionCount, expected 168 got %v", offset_of(SolverContactDesc, frictionCount))
    testing.expectf(t, offset_of(SolverContactDesc, contactForces) == 176, "Wrong offset for SolverContactDesc.contactForces, expected 176 got %v", offset_of(SolverContactDesc, contactForces))
    testing.expectf(t, offset_of(SolverContactDesc, startFrictionPatchIndex) == 184, "Wrong offset for SolverContactDesc.startFrictionPatchIndex, expected 184 got %v", offset_of(SolverContactDesc, startFrictionPatchIndex))
    testing.expectf(t, offset_of(SolverContactDesc, numFrictionPatches) == 188, "Wrong offset for SolverContactDesc.numFrictionPatches, expected 188 got %v", offset_of(SolverContactDesc, numFrictionPatches))
    testing.expectf(t, offset_of(SolverContactDesc, startContactPatchIndex) == 192, "Wrong offset for SolverContactDesc.startContactPatchIndex, expected 192 got %v", offset_of(SolverContactDesc, startContactPatchIndex))
    testing.expectf(t, offset_of(SolverContactDesc, numContactPatches) == 196, "Wrong offset for SolverContactDesc.numContactPatches, expected 196 got %v", offset_of(SolverContactDesc, numContactPatches))
    testing.expectf(t, offset_of(SolverContactDesc, axisConstraintCount) == 198, "Wrong offset for SolverContactDesc.axisConstraintCount, expected 198 got %v", offset_of(SolverContactDesc, axisConstraintCount))
    testing.expectf(t, offset_of(SolverContactDesc, offsetSlop) == 200, "Wrong offset for SolverContactDesc.offsetSlop, expected 200 got %v", offset_of(SolverContactDesc, offsetSlop))
    testing.expectf(t, size_of(SolverContactDesc) == 208, "Wrong size for type SolverContactDesc, expected 208 got %v", size_of(SolverContactDesc))
}

@(test)
test_layout_ConstraintAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ConstraintAllocator) == 8, "Wrong size for type ConstraintAllocator, expected 8 got %v", size_of(ConstraintAllocator))
}

@(test)
test_layout_ArticulationLimit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationLimit, high) == 4, "Wrong offset for ArticulationLimit.high, expected 4 got %v", offset_of(ArticulationLimit, high))
    testing.expectf(t, size_of(ArticulationLimit) == 8, "Wrong size for type ArticulationLimit, expected 8 got %v", size_of(ArticulationLimit))
}

@(test)
test_layout_ArticulationDrive :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationDrive, damping) == 4, "Wrong offset for ArticulationDrive.damping, expected 4 got %v", offset_of(ArticulationDrive, damping))
    testing.expectf(t, offset_of(ArticulationDrive, maxForce) == 8, "Wrong offset for ArticulationDrive.maxForce, expected 8 got %v", offset_of(ArticulationDrive, maxForce))
    testing.expectf(t, offset_of(ArticulationDrive, driveType) == 12, "Wrong offset for ArticulationDrive.driveType, expected 12 got %v", offset_of(ArticulationDrive, driveType))
    testing.expectf(t, size_of(ArticulationDrive) == 16, "Wrong size for type ArticulationDrive, expected 16 got %v", size_of(ArticulationDrive))
}

@(test)
test_layout_TGSSolverBodyVel :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TGSSolverBodyVel, nbStaticInteractions) == 12, "Wrong offset for TGSSolverBodyVel.nbStaticInteractions, expected 12 got %v", offset_of(TGSSolverBodyVel, nbStaticInteractions))
    testing.expectf(t, offset_of(TGSSolverBodyVel, maxDynamicPartition) == 14, "Wrong offset for TGSSolverBodyVel.maxDynamicPartition, expected 14 got %v", offset_of(TGSSolverBodyVel, maxDynamicPartition))
    testing.expectf(t, offset_of(TGSSolverBodyVel, angularVelocity) == 16, "Wrong offset for TGSSolverBodyVel.angularVelocity, expected 16 got %v", offset_of(TGSSolverBodyVel, angularVelocity))
    testing.expectf(t, offset_of(TGSSolverBodyVel, partitionMask) == 28, "Wrong offset for TGSSolverBodyVel.partitionMask, expected 28 got %v", offset_of(TGSSolverBodyVel, partitionMask))
    testing.expectf(t, offset_of(TGSSolverBodyVel, deltaAngDt) == 32, "Wrong offset for TGSSolverBodyVel.deltaAngDt, expected 32 got %v", offset_of(TGSSolverBodyVel, deltaAngDt))
    testing.expectf(t, offset_of(TGSSolverBodyVel, maxAngVel) == 44, "Wrong offset for TGSSolverBodyVel.maxAngVel, expected 44 got %v", offset_of(TGSSolverBodyVel, maxAngVel))
    testing.expectf(t, offset_of(TGSSolverBodyVel, deltaLinDt) == 48, "Wrong offset for TGSSolverBodyVel.deltaLinDt, expected 48 got %v", offset_of(TGSSolverBodyVel, deltaLinDt))
    testing.expectf(t, offset_of(TGSSolverBodyVel, lockFlags) == 60, "Wrong offset for TGSSolverBodyVel.lockFlags, expected 60 got %v", offset_of(TGSSolverBodyVel, lockFlags))
    testing.expectf(t, offset_of(TGSSolverBodyVel, isKinematic) == 62, "Wrong offset for TGSSolverBodyVel.isKinematic, expected 62 got %v", offset_of(TGSSolverBodyVel, isKinematic))
    testing.expectf(t, offset_of(TGSSolverBodyVel, pad) == 63, "Wrong offset for TGSSolverBodyVel.pad, expected 63 got %v", offset_of(TGSSolverBodyVel, pad))
    testing.expectf(t, size_of(TGSSolverBodyVel) == 64, "Wrong size for type TGSSolverBodyVel, expected 64 got %v", size_of(TGSSolverBodyVel))
}

@(test)
test_layout_TGSSolverBodyTxInertia :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TGSSolverBodyTxInertia, sqrtInvInertia) == 28, "Wrong offset for TGSSolverBodyTxInertia.sqrtInvInertia, expected 28 got %v", offset_of(TGSSolverBodyTxInertia, sqrtInvInertia))
    testing.expectf(t, size_of(TGSSolverBodyTxInertia) == 64, "Wrong size for type TGSSolverBodyTxInertia, expected 64 got %v", size_of(TGSSolverBodyTxInertia))
}

@(test)
test_layout_TGSSolverBodyData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TGSSolverBodyData, maxContactImpulse) == 12, "Wrong offset for TGSSolverBodyData.maxContactImpulse, expected 12 got %v", offset_of(TGSSolverBodyData, maxContactImpulse))
    testing.expectf(t, offset_of(TGSSolverBodyData, originalAngularVelocity) == 16, "Wrong offset for TGSSolverBodyData.originalAngularVelocity, expected 16 got %v", offset_of(TGSSolverBodyData, originalAngularVelocity))
    testing.expectf(t, offset_of(TGSSolverBodyData, penBiasClamp) == 28, "Wrong offset for TGSSolverBodyData.penBiasClamp, expected 28 got %v", offset_of(TGSSolverBodyData, penBiasClamp))
    testing.expectf(t, offset_of(TGSSolverBodyData, invMass) == 32, "Wrong offset for TGSSolverBodyData.invMass, expected 32 got %v", offset_of(TGSSolverBodyData, invMass))
    testing.expectf(t, offset_of(TGSSolverBodyData, nodeIndex) == 36, "Wrong offset for TGSSolverBodyData.nodeIndex, expected 36 got %v", offset_of(TGSSolverBodyData, nodeIndex))
    testing.expectf(t, offset_of(TGSSolverBodyData, reportThreshold) == 40, "Wrong offset for TGSSolverBodyData.reportThreshold, expected 40 got %v", offset_of(TGSSolverBodyData, reportThreshold))
    testing.expectf(t, offset_of(TGSSolverBodyData, pad) == 44, "Wrong offset for TGSSolverBodyData.pad, expected 44 got %v", offset_of(TGSSolverBodyData, pad))
    testing.expectf(t, size_of(TGSSolverBodyData) == 48, "Wrong size for type TGSSolverBodyData, expected 48 got %v", size_of(TGSSolverBodyData))
}

@(test)
test_layout_TGSSolverConstraintPrepDescBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, desc) == 16, "Wrong offset for TGSSolverConstraintPrepDescBase.desc, expected 16 got %v", offset_of(TGSSolverConstraintPrepDescBase, desc))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, body0) == 24, "Wrong offset for TGSSolverConstraintPrepDescBase.body0, expected 24 got %v", offset_of(TGSSolverConstraintPrepDescBase, body0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, body1) == 32, "Wrong offset for TGSSolverConstraintPrepDescBase.body1, expected 32 got %v", offset_of(TGSSolverConstraintPrepDescBase, body1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, body0TxI) == 40, "Wrong offset for TGSSolverConstraintPrepDescBase.body0TxI, expected 40 got %v", offset_of(TGSSolverConstraintPrepDescBase, body0TxI))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, body1TxI) == 48, "Wrong offset for TGSSolverConstraintPrepDescBase.body1TxI, expected 48 got %v", offset_of(TGSSolverConstraintPrepDescBase, body1TxI))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, bodyData0) == 56, "Wrong offset for TGSSolverConstraintPrepDescBase.bodyData0, expected 56 got %v", offset_of(TGSSolverConstraintPrepDescBase, bodyData0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, bodyData1) == 64, "Wrong offset for TGSSolverConstraintPrepDescBase.bodyData1, expected 64 got %v", offset_of(TGSSolverConstraintPrepDescBase, bodyData1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, bodyFrame0) == 72, "Wrong offset for TGSSolverConstraintPrepDescBase.bodyFrame0, expected 72 got %v", offset_of(TGSSolverConstraintPrepDescBase, bodyFrame0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, bodyFrame1) == 100, "Wrong offset for TGSSolverConstraintPrepDescBase.bodyFrame1, expected 100 got %v", offset_of(TGSSolverConstraintPrepDescBase, bodyFrame1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, bodyState0) == 128, "Wrong offset for TGSSolverConstraintPrepDescBase.bodyState0, expected 128 got %v", offset_of(TGSSolverConstraintPrepDescBase, bodyState0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDescBase, bodyState1) == 132, "Wrong offset for TGSSolverConstraintPrepDescBase.bodyState1, expected 132 got %v", offset_of(TGSSolverConstraintPrepDescBase, bodyState1))
    testing.expectf(t, size_of(TGSSolverConstraintPrepDescBase) == 144, "Wrong size for type TGSSolverConstraintPrepDescBase, expected 144 got %v", size_of(TGSSolverConstraintPrepDescBase))
}

@(test)
test_layout_TGSSolverConstraintPrepDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, desc) == 16, "Wrong offset for TGSSolverConstraintPrepDesc.desc, expected 16 got %v", offset_of(TGSSolverConstraintPrepDesc, desc))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, body0) == 24, "Wrong offset for TGSSolverConstraintPrepDesc.body0, expected 24 got %v", offset_of(TGSSolverConstraintPrepDesc, body0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, body1) == 32, "Wrong offset for TGSSolverConstraintPrepDesc.body1, expected 32 got %v", offset_of(TGSSolverConstraintPrepDesc, body1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, body0TxI) == 40, "Wrong offset for TGSSolverConstraintPrepDesc.body0TxI, expected 40 got %v", offset_of(TGSSolverConstraintPrepDesc, body0TxI))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, body1TxI) == 48, "Wrong offset for TGSSolverConstraintPrepDesc.body1TxI, expected 48 got %v", offset_of(TGSSolverConstraintPrepDesc, body1TxI))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, bodyData0) == 56, "Wrong offset for TGSSolverConstraintPrepDesc.bodyData0, expected 56 got %v", offset_of(TGSSolverConstraintPrepDesc, bodyData0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, bodyData1) == 64, "Wrong offset for TGSSolverConstraintPrepDesc.bodyData1, expected 64 got %v", offset_of(TGSSolverConstraintPrepDesc, bodyData1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, bodyFrame0) == 72, "Wrong offset for TGSSolverConstraintPrepDesc.bodyFrame0, expected 72 got %v", offset_of(TGSSolverConstraintPrepDesc, bodyFrame0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, bodyFrame1) == 100, "Wrong offset for TGSSolverConstraintPrepDesc.bodyFrame1, expected 100 got %v", offset_of(TGSSolverConstraintPrepDesc, bodyFrame1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, bodyState0) == 128, "Wrong offset for TGSSolverConstraintPrepDesc.bodyState0, expected 128 got %v", offset_of(TGSSolverConstraintPrepDesc, bodyState0))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, bodyState1) == 132, "Wrong offset for TGSSolverConstraintPrepDesc.bodyState1, expected 132 got %v", offset_of(TGSSolverConstraintPrepDesc, bodyState1))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, rows) == 144, "Wrong offset for TGSSolverConstraintPrepDesc.rows, expected 144 got %v", offset_of(TGSSolverConstraintPrepDesc, rows))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, numRows) == 152, "Wrong offset for TGSSolverConstraintPrepDesc.numRows, expected 152 got %v", offset_of(TGSSolverConstraintPrepDesc, numRows))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, linBreakForce) == 156, "Wrong offset for TGSSolverConstraintPrepDesc.linBreakForce, expected 156 got %v", offset_of(TGSSolverConstraintPrepDesc, linBreakForce))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, angBreakForce) == 160, "Wrong offset for TGSSolverConstraintPrepDesc.angBreakForce, expected 160 got %v", offset_of(TGSSolverConstraintPrepDesc, angBreakForce))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, minResponseThreshold) == 164, "Wrong offset for TGSSolverConstraintPrepDesc.minResponseThreshold, expected 164 got %v", offset_of(TGSSolverConstraintPrepDesc, minResponseThreshold))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, writeback) == 168, "Wrong offset for TGSSolverConstraintPrepDesc.writeback, expected 168 got %v", offset_of(TGSSolverConstraintPrepDesc, writeback))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, disablePreprocessing) == 176, "Wrong offset for TGSSolverConstraintPrepDesc.disablePreprocessing, expected 176 got %v", offset_of(TGSSolverConstraintPrepDesc, disablePreprocessing))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, improvedSlerp) == 177, "Wrong offset for TGSSolverConstraintPrepDesc.improvedSlerp, expected 177 got %v", offset_of(TGSSolverConstraintPrepDesc, improvedSlerp))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, driveLimitsAreForces) == 178, "Wrong offset for TGSSolverConstraintPrepDesc.driveLimitsAreForces, expected 178 got %v", offset_of(TGSSolverConstraintPrepDesc, driveLimitsAreForces))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, extendedLimits) == 179, "Wrong offset for TGSSolverConstraintPrepDesc.extendedLimits, expected 179 got %v", offset_of(TGSSolverConstraintPrepDesc, extendedLimits))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, disableConstraint) == 180, "Wrong offset for TGSSolverConstraintPrepDesc.disableConstraint, expected 180 got %v", offset_of(TGSSolverConstraintPrepDesc, disableConstraint))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, body0WorldOffset) == 184, "Wrong offset for TGSSolverConstraintPrepDesc.body0WorldOffset, expected 184 got %v", offset_of(TGSSolverConstraintPrepDesc, body0WorldOffset))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, cA2w) == 200, "Wrong offset for TGSSolverConstraintPrepDesc.cA2w, expected 200 got %v", offset_of(TGSSolverConstraintPrepDesc, cA2w))
    testing.expectf(t, offset_of(TGSSolverConstraintPrepDesc, cB2w) == 216, "Wrong offset for TGSSolverConstraintPrepDesc.cB2w, expected 216 got %v", offset_of(TGSSolverConstraintPrepDesc, cB2w))
    testing.expectf(t, size_of(TGSSolverConstraintPrepDesc) == 240, "Wrong size for type TGSSolverConstraintPrepDesc, expected 240 got %v", size_of(TGSSolverConstraintPrepDesc))
}

@(test)
test_layout_TGSSolverContactDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TGSSolverContactDesc, desc) == 16, "Wrong offset for TGSSolverContactDesc.desc, expected 16 got %v", offset_of(TGSSolverContactDesc, desc))
    testing.expectf(t, offset_of(TGSSolverContactDesc, body0) == 24, "Wrong offset for TGSSolverContactDesc.body0, expected 24 got %v", offset_of(TGSSolverContactDesc, body0))
    testing.expectf(t, offset_of(TGSSolverContactDesc, body1) == 32, "Wrong offset for TGSSolverContactDesc.body1, expected 32 got %v", offset_of(TGSSolverContactDesc, body1))
    testing.expectf(t, offset_of(TGSSolverContactDesc, body0TxI) == 40, "Wrong offset for TGSSolverContactDesc.body0TxI, expected 40 got %v", offset_of(TGSSolverContactDesc, body0TxI))
    testing.expectf(t, offset_of(TGSSolverContactDesc, body1TxI) == 48, "Wrong offset for TGSSolverContactDesc.body1TxI, expected 48 got %v", offset_of(TGSSolverContactDesc, body1TxI))
    testing.expectf(t, offset_of(TGSSolverContactDesc, bodyData0) == 56, "Wrong offset for TGSSolverContactDesc.bodyData0, expected 56 got %v", offset_of(TGSSolverContactDesc, bodyData0))
    testing.expectf(t, offset_of(TGSSolverContactDesc, bodyData1) == 64, "Wrong offset for TGSSolverContactDesc.bodyData1, expected 64 got %v", offset_of(TGSSolverContactDesc, bodyData1))
    testing.expectf(t, offset_of(TGSSolverContactDesc, bodyFrame0) == 72, "Wrong offset for TGSSolverContactDesc.bodyFrame0, expected 72 got %v", offset_of(TGSSolverContactDesc, bodyFrame0))
    testing.expectf(t, offset_of(TGSSolverContactDesc, bodyFrame1) == 100, "Wrong offset for TGSSolverContactDesc.bodyFrame1, expected 100 got %v", offset_of(TGSSolverContactDesc, bodyFrame1))
    testing.expectf(t, offset_of(TGSSolverContactDesc, bodyState0) == 128, "Wrong offset for TGSSolverContactDesc.bodyState0, expected 128 got %v", offset_of(TGSSolverContactDesc, bodyState0))
    testing.expectf(t, offset_of(TGSSolverContactDesc, bodyState1) == 132, "Wrong offset for TGSSolverContactDesc.bodyState1, expected 132 got %v", offset_of(TGSSolverContactDesc, bodyState1))
    testing.expectf(t, offset_of(TGSSolverContactDesc, shapeInteraction) == 144, "Wrong offset for TGSSolverContactDesc.shapeInteraction, expected 144 got %v", offset_of(TGSSolverContactDesc, shapeInteraction))
    testing.expectf(t, offset_of(TGSSolverContactDesc, contacts) == 152, "Wrong offset for TGSSolverContactDesc.contacts, expected 152 got %v", offset_of(TGSSolverContactDesc, contacts))
    testing.expectf(t, offset_of(TGSSolverContactDesc, numContacts) == 160, "Wrong offset for TGSSolverContactDesc.numContacts, expected 160 got %v", offset_of(TGSSolverContactDesc, numContacts))
    testing.expectf(t, offset_of(TGSSolverContactDesc, hasMaxImpulse) == 164, "Wrong offset for TGSSolverContactDesc.hasMaxImpulse, expected 164 got %v", offset_of(TGSSolverContactDesc, hasMaxImpulse))
    testing.expectf(t, offset_of(TGSSolverContactDesc, disableStrongFriction) == 165, "Wrong offset for TGSSolverContactDesc.disableStrongFriction, expected 165 got %v", offset_of(TGSSolverContactDesc, disableStrongFriction))
    testing.expectf(t, offset_of(TGSSolverContactDesc, hasForceThresholds) == 166, "Wrong offset for TGSSolverContactDesc.hasForceThresholds, expected 166 got %v", offset_of(TGSSolverContactDesc, hasForceThresholds))
    testing.expectf(t, offset_of(TGSSolverContactDesc, restDistance) == 168, "Wrong offset for TGSSolverContactDesc.restDistance, expected 168 got %v", offset_of(TGSSolverContactDesc, restDistance))
    testing.expectf(t, offset_of(TGSSolverContactDesc, maxCCDSeparation) == 172, "Wrong offset for TGSSolverContactDesc.maxCCDSeparation, expected 172 got %v", offset_of(TGSSolverContactDesc, maxCCDSeparation))
    testing.expectf(t, offset_of(TGSSolverContactDesc, frictionPtr) == 176, "Wrong offset for TGSSolverContactDesc.frictionPtr, expected 176 got %v", offset_of(TGSSolverContactDesc, frictionPtr))
    testing.expectf(t, offset_of(TGSSolverContactDesc, frictionCount) == 184, "Wrong offset for TGSSolverContactDesc.frictionCount, expected 184 got %v", offset_of(TGSSolverContactDesc, frictionCount))
    testing.expectf(t, offset_of(TGSSolverContactDesc, contactForces) == 192, "Wrong offset for TGSSolverContactDesc.contactForces, expected 192 got %v", offset_of(TGSSolverContactDesc, contactForces))
    testing.expectf(t, offset_of(TGSSolverContactDesc, startFrictionPatchIndex) == 200, "Wrong offset for TGSSolverContactDesc.startFrictionPatchIndex, expected 200 got %v", offset_of(TGSSolverContactDesc, startFrictionPatchIndex))
    testing.expectf(t, offset_of(TGSSolverContactDesc, numFrictionPatches) == 204, "Wrong offset for TGSSolverContactDesc.numFrictionPatches, expected 204 got %v", offset_of(TGSSolverContactDesc, numFrictionPatches))
    testing.expectf(t, offset_of(TGSSolverContactDesc, startContactPatchIndex) == 208, "Wrong offset for TGSSolverContactDesc.startContactPatchIndex, expected 208 got %v", offset_of(TGSSolverContactDesc, startContactPatchIndex))
    testing.expectf(t, offset_of(TGSSolverContactDesc, numContactPatches) == 212, "Wrong offset for TGSSolverContactDesc.numContactPatches, expected 212 got %v", offset_of(TGSSolverContactDesc, numContactPatches))
    testing.expectf(t, offset_of(TGSSolverContactDesc, axisConstraintCount) == 214, "Wrong offset for TGSSolverContactDesc.axisConstraintCount, expected 214 got %v", offset_of(TGSSolverContactDesc, axisConstraintCount))
    testing.expectf(t, offset_of(TGSSolverContactDesc, maxImpulse) == 216, "Wrong offset for TGSSolverContactDesc.maxImpulse, expected 216 got %v", offset_of(TGSSolverContactDesc, maxImpulse))
    testing.expectf(t, offset_of(TGSSolverContactDesc, torsionalPatchRadius) == 220, "Wrong offset for TGSSolverContactDesc.torsionalPatchRadius, expected 220 got %v", offset_of(TGSSolverContactDesc, torsionalPatchRadius))
    testing.expectf(t, offset_of(TGSSolverContactDesc, minTorsionalPatchRadius) == 224, "Wrong offset for TGSSolverContactDesc.minTorsionalPatchRadius, expected 224 got %v", offset_of(TGSSolverContactDesc, minTorsionalPatchRadius))
    testing.expectf(t, offset_of(TGSSolverContactDesc, offsetSlop) == 228, "Wrong offset for TGSSolverContactDesc.offsetSlop, expected 228 got %v", offset_of(TGSSolverContactDesc, offsetSlop))
    testing.expectf(t, size_of(TGSSolverContactDesc) == 240, "Wrong size for type TGSSolverContactDesc, expected 240 got %v", size_of(TGSSolverContactDesc))
}

@(test)
test_layout_ArticulationTendonLimit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationTendonLimit, highLimit) == 4, "Wrong offset for ArticulationTendonLimit.highLimit, expected 4 got %v", offset_of(ArticulationTendonLimit, highLimit))
    testing.expectf(t, size_of(ArticulationTendonLimit) == 8, "Wrong size for type ArticulationTendonLimit, expected 8 got %v", size_of(ArticulationTendonLimit))
}

@(test)
test_layout_ArticulationAttachment :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationAttachment, userData) == 16, "Wrong offset for ArticulationAttachment.userData, expected 16 got %v", offset_of(ArticulationAttachment, userData))
    testing.expectf(t, size_of(ArticulationAttachment) == 24, "Wrong size for type ArticulationAttachment, expected 24 got %v", size_of(ArticulationAttachment))
}

@(test)
test_layout_ArticulationTendonJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationTendonJoint, userData) == 16, "Wrong offset for ArticulationTendonJoint.userData, expected 16 got %v", offset_of(ArticulationTendonJoint, userData))
    testing.expectf(t, size_of(ArticulationTendonJoint) == 24, "Wrong size for type ArticulationTendonJoint, expected 24 got %v", size_of(ArticulationTendonJoint))
}

@(test)
test_layout_ArticulationTendon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationTendon, userData) == 16, "Wrong offset for ArticulationTendon.userData, expected 16 got %v", offset_of(ArticulationTendon, userData))
    testing.expectf(t, size_of(ArticulationTendon) == 24, "Wrong size for type ArticulationTendon, expected 24 got %v", size_of(ArticulationTendon))
}

@(test)
test_layout_ArticulationSpatialTendon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ArticulationSpatialTendon) == 24, "Wrong size for type ArticulationSpatialTendon, expected 24 got %v", size_of(ArticulationSpatialTendon))
}

@(test)
test_layout_ArticulationFixedTendon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ArticulationFixedTendon) == 24, "Wrong size for type ArticulationFixedTendon, expected 24 got %v", size_of(ArticulationFixedTendon))
}

@(test)
test_layout_SpatialForce :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SpatialForce, pad0) == 12, "Wrong offset for SpatialForce.pad0, expected 12 got %v", offset_of(SpatialForce, pad0))
    testing.expectf(t, offset_of(SpatialForce, torque) == 16, "Wrong offset for SpatialForce.torque, expected 16 got %v", offset_of(SpatialForce, torque))
    testing.expectf(t, offset_of(SpatialForce, pad1) == 28, "Wrong offset for SpatialForce.pad1, expected 28 got %v", offset_of(SpatialForce, pad1))
    testing.expectf(t, size_of(SpatialForce) == 32, "Wrong size for type SpatialForce, expected 32 got %v", size_of(SpatialForce))
}

@(test)
test_layout_SpatialVelocity :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SpatialVelocity, pad0) == 12, "Wrong offset for SpatialVelocity.pad0, expected 12 got %v", offset_of(SpatialVelocity, pad0))
    testing.expectf(t, offset_of(SpatialVelocity, angular) == 16, "Wrong offset for SpatialVelocity.angular, expected 16 got %v", offset_of(SpatialVelocity, angular))
    testing.expectf(t, offset_of(SpatialVelocity, pad1) == 28, "Wrong offset for SpatialVelocity.pad1, expected 28 got %v", offset_of(SpatialVelocity, pad1))
    testing.expectf(t, size_of(SpatialVelocity) == 32, "Wrong size for type SpatialVelocity, expected 32 got %v", size_of(SpatialVelocity))
}

@(test)
test_layout_ArticulationRootLinkData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationRootLinkData, worldLinVel) == 28, "Wrong offset for ArticulationRootLinkData.worldLinVel, expected 28 got %v", offset_of(ArticulationRootLinkData, worldLinVel))
    testing.expectf(t, offset_of(ArticulationRootLinkData, worldAngVel) == 40, "Wrong offset for ArticulationRootLinkData.worldAngVel, expected 40 got %v", offset_of(ArticulationRootLinkData, worldAngVel))
    testing.expectf(t, offset_of(ArticulationRootLinkData, worldLinAccel) == 52, "Wrong offset for ArticulationRootLinkData.worldLinAccel, expected 52 got %v", offset_of(ArticulationRootLinkData, worldLinAccel))
    testing.expectf(t, offset_of(ArticulationRootLinkData, worldAngAccel) == 64, "Wrong offset for ArticulationRootLinkData.worldAngAccel, expected 64 got %v", offset_of(ArticulationRootLinkData, worldAngAccel))
    testing.expectf(t, size_of(ArticulationRootLinkData) == 76, "Wrong size for type ArticulationRootLinkData, expected 76 got %v", size_of(ArticulationRootLinkData))
}

@(test)
test_layout_ArticulationCache :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationCache, denseJacobian) == 8, "Wrong offset for ArticulationCache.denseJacobian, expected 8 got %v", offset_of(ArticulationCache, denseJacobian))
    testing.expectf(t, offset_of(ArticulationCache, massMatrix) == 16, "Wrong offset for ArticulationCache.massMatrix, expected 16 got %v", offset_of(ArticulationCache, massMatrix))
    testing.expectf(t, offset_of(ArticulationCache, jointVelocity) == 24, "Wrong offset for ArticulationCache.jointVelocity, expected 24 got %v", offset_of(ArticulationCache, jointVelocity))
    testing.expectf(t, offset_of(ArticulationCache, jointAcceleration) == 32, "Wrong offset for ArticulationCache.jointAcceleration, expected 32 got %v", offset_of(ArticulationCache, jointAcceleration))
    testing.expectf(t, offset_of(ArticulationCache, jointPosition) == 40, "Wrong offset for ArticulationCache.jointPosition, expected 40 got %v", offset_of(ArticulationCache, jointPosition))
    testing.expectf(t, offset_of(ArticulationCache, jointForce) == 48, "Wrong offset for ArticulationCache.jointForce, expected 48 got %v", offset_of(ArticulationCache, jointForce))
    testing.expectf(t, offset_of(ArticulationCache, jointSolverForces) == 56, "Wrong offset for ArticulationCache.jointSolverForces, expected 56 got %v", offset_of(ArticulationCache, jointSolverForces))
    testing.expectf(t, offset_of(ArticulationCache, linkVelocity) == 64, "Wrong offset for ArticulationCache.linkVelocity, expected 64 got %v", offset_of(ArticulationCache, linkVelocity))
    testing.expectf(t, offset_of(ArticulationCache, linkAcceleration) == 72, "Wrong offset for ArticulationCache.linkAcceleration, expected 72 got %v", offset_of(ArticulationCache, linkAcceleration))
    testing.expectf(t, offset_of(ArticulationCache, rootLinkData) == 80, "Wrong offset for ArticulationCache.rootLinkData, expected 80 got %v", offset_of(ArticulationCache, rootLinkData))
    testing.expectf(t, offset_of(ArticulationCache, sensorForces) == 88, "Wrong offset for ArticulationCache.sensorForces, expected 88 got %v", offset_of(ArticulationCache, sensorForces))
    testing.expectf(t, offset_of(ArticulationCache, coefficientMatrix) == 96, "Wrong offset for ArticulationCache.coefficientMatrix, expected 96 got %v", offset_of(ArticulationCache, coefficientMatrix))
    testing.expectf(t, offset_of(ArticulationCache, lambda) == 104, "Wrong offset for ArticulationCache.lambda, expected 104 got %v", offset_of(ArticulationCache, lambda))
    testing.expectf(t, offset_of(ArticulationCache, scratchMemory) == 112, "Wrong offset for ArticulationCache.scratchMemory, expected 112 got %v", offset_of(ArticulationCache, scratchMemory))
    testing.expectf(t, offset_of(ArticulationCache, scratchAllocator) == 120, "Wrong offset for ArticulationCache.scratchAllocator, expected 120 got %v", offset_of(ArticulationCache, scratchAllocator))
    testing.expectf(t, offset_of(ArticulationCache, version) == 128, "Wrong offset for ArticulationCache.version, expected 128 got %v", offset_of(ArticulationCache, version))
    testing.expectf(t, size_of(ArticulationCache) == 136, "Wrong size for type ArticulationCache, expected 136 got %v", size_of(ArticulationCache))
}

@(test)
test_layout_ArticulationSensor :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationSensor, userData) == 16, "Wrong offset for ArticulationSensor.userData, expected 16 got %v", offset_of(ArticulationSensor, userData))
    testing.expectf(t, size_of(ArticulationSensor) == 24, "Wrong size for type ArticulationSensor, expected 24 got %v", size_of(ArticulationSensor))
}

@(test)
test_layout_ArticulationReducedCoordinate :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationReducedCoordinate, userData) == 16, "Wrong offset for ArticulationReducedCoordinate.userData, expected 16 got %v", offset_of(ArticulationReducedCoordinate, userData))
    testing.expectf(t, size_of(ArticulationReducedCoordinate) == 24, "Wrong size for type ArticulationReducedCoordinate, expected 24 got %v", size_of(ArticulationReducedCoordinate))
}

@(test)
test_layout_ArticulationJointReducedCoordinate :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ArticulationJointReducedCoordinate, userData) == 16, "Wrong offset for ArticulationJointReducedCoordinate.userData, expected 16 got %v", offset_of(ArticulationJointReducedCoordinate, userData))
    testing.expectf(t, size_of(ArticulationJointReducedCoordinate) == 24, "Wrong size for type ArticulationJointReducedCoordinate, expected 24 got %v", size_of(ArticulationJointReducedCoordinate))
}

@(test)
test_layout_Shape :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Shape, userData) == 16, "Wrong offset for Shape.userData, expected 16 got %v", offset_of(Shape, userData))
    testing.expectf(t, size_of(Shape) == 24, "Wrong size for type Shape, expected 24 got %v", size_of(Shape))
}

@(test)
test_layout_RigidActor :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RigidActor) == 24, "Wrong size for type RigidActor, expected 24 got %v", size_of(RigidActor))
}

@(test)
test_layout_NodeIndex :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(NodeIndex) == 8, "Wrong size for type NodeIndex, expected 8 got %v", size_of(NodeIndex))
}

@(test)
test_layout_RigidBody :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RigidBody) == 24, "Wrong size for type RigidBody, expected 24 got %v", size_of(RigidBody))
}

@(test)
test_layout_ArticulationLink :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ArticulationLink) == 24, "Wrong size for type ArticulationLink, expected 24 got %v", size_of(ArticulationLink))
}

@(test)
test_layout_ConeLimitedConstraint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConeLimitedConstraint, mAngle) == 12, "Wrong offset for ConeLimitedConstraint.mAngle, expected 12 got %v", offset_of(ConeLimitedConstraint, mAngle))
    testing.expectf(t, offset_of(ConeLimitedConstraint, mLowLimit) == 16, "Wrong offset for ConeLimitedConstraint.mLowLimit, expected 16 got %v", offset_of(ConeLimitedConstraint, mLowLimit))
    testing.expectf(t, offset_of(ConeLimitedConstraint, mHighLimit) == 20, "Wrong offset for ConeLimitedConstraint.mHighLimit, expected 20 got %v", offset_of(ConeLimitedConstraint, mHighLimit))
    testing.expectf(t, size_of(ConeLimitedConstraint) == 24, "Wrong size for type ConeLimitedConstraint, expected 24 got %v", size_of(ConeLimitedConstraint))
}

@(test)
test_layout_ConeLimitParams :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConeLimitParams, axisAngle) == 16, "Wrong offset for ConeLimitParams.axisAngle, expected 16 got %v", offset_of(ConeLimitParams, axisAngle))
    testing.expectf(t, size_of(ConeLimitParams) == 32, "Wrong size for type ConeLimitParams, expected 32 got %v", size_of(ConeLimitParams))
}

@(test)
test_layout_ConstraintShaderTable :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConstraintShaderTable, visualize) == 16, "Wrong offset for ConstraintShaderTable.visualize, expected 16 got %v", offset_of(ConstraintShaderTable, visualize))
    testing.expectf(t, offset_of(ConstraintShaderTable, flag) == 24, "Wrong offset for ConstraintShaderTable.flag, expected 24 got %v", offset_of(ConstraintShaderTable, flag))
    testing.expectf(t, size_of(ConstraintShaderTable) == 32, "Wrong size for type ConstraintShaderTable, expected 32 got %v", size_of(ConstraintShaderTable))
}

@(test)
test_layout_Constraint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Constraint, userData) == 16, "Wrong offset for Constraint.userData, expected 16 got %v", offset_of(Constraint, userData))
    testing.expectf(t, size_of(Constraint) == 24, "Wrong size for type Constraint, expected 24 got %v", size_of(Constraint))
}

@(test)
test_layout_MassModificationProps :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(MassModificationProps, mInvInertiaScale0) == 4, "Wrong offset for MassModificationProps.mInvInertiaScale0, expected 4 got %v", offset_of(MassModificationProps, mInvInertiaScale0))
    testing.expectf(t, offset_of(MassModificationProps, mInvMassScale1) == 8, "Wrong offset for MassModificationProps.mInvMassScale1, expected 8 got %v", offset_of(MassModificationProps, mInvMassScale1))
    testing.expectf(t, offset_of(MassModificationProps, mInvInertiaScale1) == 12, "Wrong offset for MassModificationProps.mInvInertiaScale1, expected 12 got %v", offset_of(MassModificationProps, mInvInertiaScale1))
    testing.expectf(t, size_of(MassModificationProps) == 16, "Wrong size for type MassModificationProps, expected 16 got %v", size_of(MassModificationProps))
}

@(test)
test_layout_ContactPatch :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPatch, normal) == 16, "Wrong offset for ContactPatch.normal, expected 16 got %v", offset_of(ContactPatch, normal))
    testing.expectf(t, offset_of(ContactPatch, restitution) == 28, "Wrong offset for ContactPatch.restitution, expected 28 got %v", offset_of(ContactPatch, restitution))
    testing.expectf(t, offset_of(ContactPatch, dynamicFriction) == 32, "Wrong offset for ContactPatch.dynamicFriction, expected 32 got %v", offset_of(ContactPatch, dynamicFriction))
    testing.expectf(t, offset_of(ContactPatch, staticFriction) == 36, "Wrong offset for ContactPatch.staticFriction, expected 36 got %v", offset_of(ContactPatch, staticFriction))
    testing.expectf(t, offset_of(ContactPatch, damping) == 40, "Wrong offset for ContactPatch.damping, expected 40 got %v", offset_of(ContactPatch, damping))
    testing.expectf(t, offset_of(ContactPatch, startContactIndex) == 44, "Wrong offset for ContactPatch.startContactIndex, expected 44 got %v", offset_of(ContactPatch, startContactIndex))
    testing.expectf(t, offset_of(ContactPatch, nbContacts) == 46, "Wrong offset for ContactPatch.nbContacts, expected 46 got %v", offset_of(ContactPatch, nbContacts))
    testing.expectf(t, offset_of(ContactPatch, materialFlags) == 47, "Wrong offset for ContactPatch.materialFlags, expected 47 got %v", offset_of(ContactPatch, materialFlags))
    testing.expectf(t, offset_of(ContactPatch, internalFlags) == 48, "Wrong offset for ContactPatch.internalFlags, expected 48 got %v", offset_of(ContactPatch, internalFlags))
    testing.expectf(t, offset_of(ContactPatch, materialIndex0) == 50, "Wrong offset for ContactPatch.materialIndex0, expected 50 got %v", offset_of(ContactPatch, materialIndex0))
    testing.expectf(t, offset_of(ContactPatch, materialIndex1) == 52, "Wrong offset for ContactPatch.materialIndex1, expected 52 got %v", offset_of(ContactPatch, materialIndex1))
    testing.expectf(t, size_of(ContactPatch) == 64, "Wrong size for type ContactPatch, expected 64 got %v", size_of(ContactPatch))
}

@(test)
test_layout_Contact :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Contact, separation) == 12, "Wrong offset for Contact.separation, expected 12 got %v", offset_of(Contact, separation))
    testing.expectf(t, size_of(Contact) == 16, "Wrong size for type Contact, expected 16 got %v", size_of(Contact))
}

@(test)
test_layout_ExtendedContact :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ExtendedContact, targetVelocity) == 16, "Wrong offset for ExtendedContact.targetVelocity, expected 16 got %v", offset_of(ExtendedContact, targetVelocity))
    testing.expectf(t, offset_of(ExtendedContact, maxImpulse) == 28, "Wrong offset for ExtendedContact.maxImpulse, expected 28 got %v", offset_of(ExtendedContact, maxImpulse))
    testing.expectf(t, size_of(ExtendedContact) == 32, "Wrong size for type ExtendedContact, expected 32 got %v", size_of(ExtendedContact))
}

@(test)
test_layout_ModifiableContact :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ModifiableContact, normal) == 32, "Wrong offset for ModifiableContact.normal, expected 32 got %v", offset_of(ModifiableContact, normal))
    testing.expectf(t, offset_of(ModifiableContact, restitution) == 44, "Wrong offset for ModifiableContact.restitution, expected 44 got %v", offset_of(ModifiableContact, restitution))
    testing.expectf(t, offset_of(ModifiableContact, materialFlags) == 48, "Wrong offset for ModifiableContact.materialFlags, expected 48 got %v", offset_of(ModifiableContact, materialFlags))
    testing.expectf(t, offset_of(ModifiableContact, materialIndex0) == 52, "Wrong offset for ModifiableContact.materialIndex0, expected 52 got %v", offset_of(ModifiableContact, materialIndex0))
    testing.expectf(t, offset_of(ModifiableContact, materialIndex1) == 54, "Wrong offset for ModifiableContact.materialIndex1, expected 54 got %v", offset_of(ModifiableContact, materialIndex1))
    testing.expectf(t, offset_of(ModifiableContact, staticFriction) == 56, "Wrong offset for ModifiableContact.staticFriction, expected 56 got %v", offset_of(ModifiableContact, staticFriction))
    testing.expectf(t, offset_of(ModifiableContact, dynamicFriction) == 60, "Wrong offset for ModifiableContact.dynamicFriction, expected 60 got %v", offset_of(ModifiableContact, dynamicFriction))
    testing.expectf(t, size_of(ModifiableContact) == 64, "Wrong size for type ModifiableContact, expected 64 got %v", size_of(ModifiableContact))
}

@(test)
test_layout_ContactStreamIterator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactStreamIterator, patch) == 16, "Wrong offset for ContactStreamIterator.patch, expected 16 got %v", offset_of(ContactStreamIterator, patch))
    testing.expectf(t, offset_of(ContactStreamIterator, contact) == 24, "Wrong offset for ContactStreamIterator.contact, expected 24 got %v", offset_of(ContactStreamIterator, contact))
    testing.expectf(t, offset_of(ContactStreamIterator, faceIndice) == 32, "Wrong offset for ContactStreamIterator.faceIndice, expected 32 got %v", offset_of(ContactStreamIterator, faceIndice))
    testing.expectf(t, offset_of(ContactStreamIterator, totalPatches) == 40, "Wrong offset for ContactStreamIterator.totalPatches, expected 40 got %v", offset_of(ContactStreamIterator, totalPatches))
    testing.expectf(t, offset_of(ContactStreamIterator, totalContacts) == 44, "Wrong offset for ContactStreamIterator.totalContacts, expected 44 got %v", offset_of(ContactStreamIterator, totalContacts))
    testing.expectf(t, offset_of(ContactStreamIterator, nextContactIndex) == 48, "Wrong offset for ContactStreamIterator.nextContactIndex, expected 48 got %v", offset_of(ContactStreamIterator, nextContactIndex))
    testing.expectf(t, offset_of(ContactStreamIterator, nextPatchIndex) == 52, "Wrong offset for ContactStreamIterator.nextPatchIndex, expected 52 got %v", offset_of(ContactStreamIterator, nextPatchIndex))
    testing.expectf(t, offset_of(ContactStreamIterator, contactPatchHeaderSize) == 56, "Wrong offset for ContactStreamIterator.contactPatchHeaderSize, expected 56 got %v", offset_of(ContactStreamIterator, contactPatchHeaderSize))
    testing.expectf(t, offset_of(ContactStreamIterator, contactPointSize) == 60, "Wrong offset for ContactStreamIterator.contactPointSize, expected 60 got %v", offset_of(ContactStreamIterator, contactPointSize))
    testing.expectf(t, offset_of(ContactStreamIterator, mStreamFormat) == 64, "Wrong offset for ContactStreamIterator.mStreamFormat, expected 64 got %v", offset_of(ContactStreamIterator, mStreamFormat))
    testing.expectf(t, offset_of(ContactStreamIterator, forceNoResponse) == 68, "Wrong offset for ContactStreamIterator.forceNoResponse, expected 68 got %v", offset_of(ContactStreamIterator, forceNoResponse))
    testing.expectf(t, offset_of(ContactStreamIterator, pointStepped) == 72, "Wrong offset for ContactStreamIterator.pointStepped, expected 72 got %v", offset_of(ContactStreamIterator, pointStepped))
    testing.expectf(t, offset_of(ContactStreamIterator, hasFaceIndices) == 76, "Wrong offset for ContactStreamIterator.hasFaceIndices, expected 76 got %v", offset_of(ContactStreamIterator, hasFaceIndices))
    testing.expectf(t, size_of(ContactStreamIterator) == 80, "Wrong size for type ContactStreamIterator, expected 80 got %v", size_of(ContactStreamIterator))
}

@(test)
test_layout_GpuContactPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GpuContactPair, contactPoints) == 8, "Wrong offset for GpuContactPair.contactPoints, expected 8 got %v", offset_of(GpuContactPair, contactPoints))
    testing.expectf(t, offset_of(GpuContactPair, contactForces) == 16, "Wrong offset for GpuContactPair.contactForces, expected 16 got %v", offset_of(GpuContactPair, contactForces))
    testing.expectf(t, offset_of(GpuContactPair, transformCacheRef0) == 24, "Wrong offset for GpuContactPair.transformCacheRef0, expected 24 got %v", offset_of(GpuContactPair, transformCacheRef0))
    testing.expectf(t, offset_of(GpuContactPair, transformCacheRef1) == 28, "Wrong offset for GpuContactPair.transformCacheRef1, expected 28 got %v", offset_of(GpuContactPair, transformCacheRef1))
    testing.expectf(t, offset_of(GpuContactPair, nodeIndex0) == 32, "Wrong offset for GpuContactPair.nodeIndex0, expected 32 got %v", offset_of(GpuContactPair, nodeIndex0))
    testing.expectf(t, offset_of(GpuContactPair, nodeIndex1) == 40, "Wrong offset for GpuContactPair.nodeIndex1, expected 40 got %v", offset_of(GpuContactPair, nodeIndex1))
    testing.expectf(t, offset_of(GpuContactPair, actor0) == 48, "Wrong offset for GpuContactPair.actor0, expected 48 got %v", offset_of(GpuContactPair, actor0))
    testing.expectf(t, offset_of(GpuContactPair, actor1) == 56, "Wrong offset for GpuContactPair.actor1, expected 56 got %v", offset_of(GpuContactPair, actor1))
    testing.expectf(t, offset_of(GpuContactPair, nbContacts) == 64, "Wrong offset for GpuContactPair.nbContacts, expected 64 got %v", offset_of(GpuContactPair, nbContacts))
    testing.expectf(t, offset_of(GpuContactPair, nbPatches) == 66, "Wrong offset for GpuContactPair.nbPatches, expected 66 got %v", offset_of(GpuContactPair, nbPatches))
    testing.expectf(t, size_of(GpuContactPair) == 72, "Wrong size for type GpuContactPair, expected 72 got %v", size_of(GpuContactPair))
}

@(test)
test_layout_ContactSet :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ContactSet) == 16, "Wrong size for type ContactSet, expected 16 got %v", size_of(ContactSet))
}

@(test)
test_layout_ContactModifyPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactModifyPair, contacts) == 88, "Wrong offset for ContactModifyPair.contacts, expected 88 got %v", offset_of(ContactModifyPair, contacts))
    testing.expectf(t, size_of(ContactModifyPair) == 104, "Wrong size for type ContactModifyPair, expected 104 got %v", size_of(ContactModifyPair))
}

@(test)
test_layout_ContactModifyCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ContactModifyCallback) == 8, "Wrong size for type ContactModifyCallback, expected 8 got %v", size_of(ContactModifyCallback))
}

@(test)
test_layout_CCDContactModifyCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CCDContactModifyCallback) == 8, "Wrong size for type CCDContactModifyCallback, expected 8 got %v", size_of(CCDContactModifyCallback))
}

@(test)
test_layout_DeletionListener :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DeletionListener) == 8, "Wrong size for type DeletionListener, expected 8 got %v", size_of(DeletionListener))
}

@(test)
test_layout_BaseMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BaseMaterial, userData) == 16, "Wrong offset for BaseMaterial.userData, expected 16 got %v", offset_of(BaseMaterial, userData))
    testing.expectf(t, size_of(BaseMaterial) == 24, "Wrong size for type BaseMaterial, expected 24 got %v", size_of(BaseMaterial))
}

@(test)
test_layout_FEMMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(FEMMaterial) == 24, "Wrong size for type FEMMaterial, expected 24 got %v", size_of(FEMMaterial))
}

@(test)
test_layout_FilterData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(FilterData, word1) == 4, "Wrong offset for FilterData.word1, expected 4 got %v", offset_of(FilterData, word1))
    testing.expectf(t, offset_of(FilterData, word2) == 8, "Wrong offset for FilterData.word2, expected 8 got %v", offset_of(FilterData, word2))
    testing.expectf(t, offset_of(FilterData, word3) == 12, "Wrong offset for FilterData.word3, expected 12 got %v", offset_of(FilterData, word3))
    testing.expectf(t, size_of(FilterData) == 16, "Wrong size for type FilterData, expected 16 got %v", size_of(FilterData))
}

@(test)
test_layout_SimulationFilterCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SimulationFilterCallback) == 8, "Wrong size for type SimulationFilterCallback, expected 8 got %v", size_of(SimulationFilterCallback))
}

@(test)
test_layout_ParticleRigidFilterPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ParticleRigidFilterPair, mID1) == 8, "Wrong offset for ParticleRigidFilterPair.mID1, expected 8 got %v", offset_of(ParticleRigidFilterPair, mID1))
    testing.expectf(t, size_of(ParticleRigidFilterPair) == 16, "Wrong size for type ParticleRigidFilterPair, expected 16 got %v", size_of(ParticleRigidFilterPair))
}

@(test)
test_layout_LockedData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(LockedData) == 8, "Wrong size for type LockedData, expected 8 got %v", size_of(LockedData))
}

@(test)
test_layout_Material :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Material) == 24, "Wrong size for type Material, expected 24 got %v", size_of(Material))
}

@(test)
test_layout_GpuParticleBufferIndexPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GpuParticleBufferIndexPair, bufferIndex) == 4, "Wrong offset for GpuParticleBufferIndexPair.bufferIndex, expected 4 got %v", offset_of(GpuParticleBufferIndexPair, bufferIndex))
    testing.expectf(t, size_of(GpuParticleBufferIndexPair) == 8, "Wrong size for type GpuParticleBufferIndexPair, expected 8 got %v", size_of(GpuParticleBufferIndexPair))
}

@(test)
test_layout_ParticleVolume :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ParticleVolume, particleIndicesOffset) == 24, "Wrong offset for ParticleVolume.particleIndicesOffset, expected 24 got %v", offset_of(ParticleVolume, particleIndicesOffset))
    testing.expectf(t, offset_of(ParticleVolume, numParticles) == 28, "Wrong offset for ParticleVolume.numParticles, expected 28 got %v", offset_of(ParticleVolume, numParticles))
    testing.expectf(t, size_of(ParticleVolume) == 32, "Wrong size for type ParticleVolume, expected 32 got %v", size_of(ParticleVolume))
}

@(test)
test_layout_DiffuseParticleParams :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(DiffuseParticleParams, lifetime) == 4, "Wrong offset for DiffuseParticleParams.lifetime, expected 4 got %v", offset_of(DiffuseParticleParams, lifetime))
    testing.expectf(t, offset_of(DiffuseParticleParams, airDrag) == 8, "Wrong offset for DiffuseParticleParams.airDrag, expected 8 got %v", offset_of(DiffuseParticleParams, airDrag))
    testing.expectf(t, offset_of(DiffuseParticleParams, bubbleDrag) == 12, "Wrong offset for DiffuseParticleParams.bubbleDrag, expected 12 got %v", offset_of(DiffuseParticleParams, bubbleDrag))
    testing.expectf(t, offset_of(DiffuseParticleParams, buoyancy) == 16, "Wrong offset for DiffuseParticleParams.buoyancy, expected 16 got %v", offset_of(DiffuseParticleParams, buoyancy))
    testing.expectf(t, offset_of(DiffuseParticleParams, kineticEnergyWeight) == 20, "Wrong offset for DiffuseParticleParams.kineticEnergyWeight, expected 20 got %v", offset_of(DiffuseParticleParams, kineticEnergyWeight))
    testing.expectf(t, offset_of(DiffuseParticleParams, pressureWeight) == 24, "Wrong offset for DiffuseParticleParams.pressureWeight, expected 24 got %v", offset_of(DiffuseParticleParams, pressureWeight))
    testing.expectf(t, offset_of(DiffuseParticleParams, divergenceWeight) == 28, "Wrong offset for DiffuseParticleParams.divergenceWeight, expected 28 got %v", offset_of(DiffuseParticleParams, divergenceWeight))
    testing.expectf(t, offset_of(DiffuseParticleParams, collisionDecay) == 32, "Wrong offset for DiffuseParticleParams.collisionDecay, expected 32 got %v", offset_of(DiffuseParticleParams, collisionDecay))
    testing.expectf(t, offset_of(DiffuseParticleParams, useAccurateVelocity) == 36, "Wrong offset for DiffuseParticleParams.useAccurateVelocity, expected 36 got %v", offset_of(DiffuseParticleParams, useAccurateVelocity))
    testing.expectf(t, size_of(DiffuseParticleParams) == 40, "Wrong size for type DiffuseParticleParams, expected 40 got %v", size_of(DiffuseParticleParams))
}

@(test)
test_layout_ParticleSpring :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ParticleSpring, ind1) == 4, "Wrong offset for ParticleSpring.ind1, expected 4 got %v", offset_of(ParticleSpring, ind1))
    testing.expectf(t, offset_of(ParticleSpring, length) == 8, "Wrong offset for ParticleSpring.length, expected 8 got %v", offset_of(ParticleSpring, length))
    testing.expectf(t, offset_of(ParticleSpring, stiffness) == 12, "Wrong offset for ParticleSpring.stiffness, expected 12 got %v", offset_of(ParticleSpring, stiffness))
    testing.expectf(t, offset_of(ParticleSpring, damping) == 16, "Wrong offset for ParticleSpring.damping, expected 16 got %v", offset_of(ParticleSpring, damping))
    testing.expectf(t, offset_of(ParticleSpring, pad) == 20, "Wrong offset for ParticleSpring.pad, expected 20 got %v", offset_of(ParticleSpring, pad))
    testing.expectf(t, size_of(ParticleSpring) == 24, "Wrong size for type ParticleSpring, expected 24 got %v", size_of(ParticleSpring))
}

@(test)
test_layout_ParticleMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ParticleMaterial) == 24, "Wrong size for type ParticleMaterial, expected 24 got %v", size_of(ParticleMaterial))
}

@(test)
test_layout_Physics :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Physics) == 8, "Wrong size for type Physics, expected 8 got %v", size_of(Physics))
}

@(test)
test_layout_ActorShape :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ActorShape, shape) == 8, "Wrong offset for ActorShape.shape, expected 8 got %v", offset_of(ActorShape, shape))
    testing.expectf(t, size_of(ActorShape) == 16, "Wrong size for type ActorShape, expected 16 got %v", size_of(ActorShape))
}

@(test)
test_layout_RaycastHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RaycastHit) == 64, "Wrong size for type RaycastHit, expected 64 got %v", size_of(RaycastHit))
}

@(test)
test_layout_OverlapHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(OverlapHit) == 24, "Wrong size for type OverlapHit, expected 24 got %v", size_of(OverlapHit))
}

@(test)
test_layout_SweepHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SweepHit) == 56, "Wrong size for type SweepHit, expected 56 got %v", size_of(SweepHit))
}

@(test)
test_layout_RaycastCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(RaycastCallback, block) == 8, "Wrong offset for RaycastCallback.block, expected 8 got %v", offset_of(RaycastCallback, block))
    testing.expectf(t, offset_of(RaycastCallback, hasBlock) == 72, "Wrong offset for RaycastCallback.hasBlock, expected 72 got %v", offset_of(RaycastCallback, hasBlock))
    testing.expectf(t, offset_of(RaycastCallback, touches) == 80, "Wrong offset for RaycastCallback.touches, expected 80 got %v", offset_of(RaycastCallback, touches))
    testing.expectf(t, offset_of(RaycastCallback, maxNbTouches) == 88, "Wrong offset for RaycastCallback.maxNbTouches, expected 88 got %v", offset_of(RaycastCallback, maxNbTouches))
    testing.expectf(t, offset_of(RaycastCallback, nbTouches) == 92, "Wrong offset for RaycastCallback.nbTouches, expected 92 got %v", offset_of(RaycastCallback, nbTouches))
    testing.expectf(t, size_of(RaycastCallback) == 96, "Wrong size for type RaycastCallback, expected 96 got %v", size_of(RaycastCallback))
}

@(test)
test_layout_OverlapCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(OverlapCallback, block) == 8, "Wrong offset for OverlapCallback.block, expected 8 got %v", offset_of(OverlapCallback, block))
    testing.expectf(t, offset_of(OverlapCallback, hasBlock) == 32, "Wrong offset for OverlapCallback.hasBlock, expected 32 got %v", offset_of(OverlapCallback, hasBlock))
    testing.expectf(t, offset_of(OverlapCallback, touches) == 40, "Wrong offset for OverlapCallback.touches, expected 40 got %v", offset_of(OverlapCallback, touches))
    testing.expectf(t, offset_of(OverlapCallback, maxNbTouches) == 48, "Wrong offset for OverlapCallback.maxNbTouches, expected 48 got %v", offset_of(OverlapCallback, maxNbTouches))
    testing.expectf(t, offset_of(OverlapCallback, nbTouches) == 52, "Wrong offset for OverlapCallback.nbTouches, expected 52 got %v", offset_of(OverlapCallback, nbTouches))
    testing.expectf(t, size_of(OverlapCallback) == 56, "Wrong size for type OverlapCallback, expected 56 got %v", size_of(OverlapCallback))
}

@(test)
test_layout_SweepCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SweepCallback, block) == 8, "Wrong offset for SweepCallback.block, expected 8 got %v", offset_of(SweepCallback, block))
    testing.expectf(t, offset_of(SweepCallback, hasBlock) == 64, "Wrong offset for SweepCallback.hasBlock, expected 64 got %v", offset_of(SweepCallback, hasBlock))
    testing.expectf(t, offset_of(SweepCallback, touches) == 72, "Wrong offset for SweepCallback.touches, expected 72 got %v", offset_of(SweepCallback, touches))
    testing.expectf(t, offset_of(SweepCallback, maxNbTouches) == 80, "Wrong offset for SweepCallback.maxNbTouches, expected 80 got %v", offset_of(SweepCallback, maxNbTouches))
    testing.expectf(t, offset_of(SweepCallback, nbTouches) == 84, "Wrong offset for SweepCallback.nbTouches, expected 84 got %v", offset_of(SweepCallback, nbTouches))
    testing.expectf(t, size_of(SweepCallback) == 88, "Wrong size for type SweepCallback, expected 88 got %v", size_of(SweepCallback))
}

@(test)
test_layout_RaycastBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(RaycastBuffer, block) == 8, "Wrong offset for RaycastBuffer.block, expected 8 got %v", offset_of(RaycastBuffer, block))
    testing.expectf(t, offset_of(RaycastBuffer, hasBlock) == 72, "Wrong offset for RaycastBuffer.hasBlock, expected 72 got %v", offset_of(RaycastBuffer, hasBlock))
    testing.expectf(t, offset_of(RaycastBuffer, touches) == 80, "Wrong offset for RaycastBuffer.touches, expected 80 got %v", offset_of(RaycastBuffer, touches))
    testing.expectf(t, offset_of(RaycastBuffer, maxNbTouches) == 88, "Wrong offset for RaycastBuffer.maxNbTouches, expected 88 got %v", offset_of(RaycastBuffer, maxNbTouches))
    testing.expectf(t, offset_of(RaycastBuffer, nbTouches) == 92, "Wrong offset for RaycastBuffer.nbTouches, expected 92 got %v", offset_of(RaycastBuffer, nbTouches))
    testing.expectf(t, size_of(RaycastBuffer) == 96, "Wrong size for type RaycastBuffer, expected 96 got %v", size_of(RaycastBuffer))
}

@(test)
test_layout_OverlapBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(OverlapBuffer, block) == 8, "Wrong offset for OverlapBuffer.block, expected 8 got %v", offset_of(OverlapBuffer, block))
    testing.expectf(t, offset_of(OverlapBuffer, hasBlock) == 32, "Wrong offset for OverlapBuffer.hasBlock, expected 32 got %v", offset_of(OverlapBuffer, hasBlock))
    testing.expectf(t, offset_of(OverlapBuffer, touches) == 40, "Wrong offset for OverlapBuffer.touches, expected 40 got %v", offset_of(OverlapBuffer, touches))
    testing.expectf(t, offset_of(OverlapBuffer, maxNbTouches) == 48, "Wrong offset for OverlapBuffer.maxNbTouches, expected 48 got %v", offset_of(OverlapBuffer, maxNbTouches))
    testing.expectf(t, offset_of(OverlapBuffer, nbTouches) == 52, "Wrong offset for OverlapBuffer.nbTouches, expected 52 got %v", offset_of(OverlapBuffer, nbTouches))
    testing.expectf(t, size_of(OverlapBuffer) == 56, "Wrong size for type OverlapBuffer, expected 56 got %v", size_of(OverlapBuffer))
}

@(test)
test_layout_SweepBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SweepBuffer, block) == 8, "Wrong offset for SweepBuffer.block, expected 8 got %v", offset_of(SweepBuffer, block))
    testing.expectf(t, offset_of(SweepBuffer, hasBlock) == 64, "Wrong offset for SweepBuffer.hasBlock, expected 64 got %v", offset_of(SweepBuffer, hasBlock))
    testing.expectf(t, offset_of(SweepBuffer, touches) == 72, "Wrong offset for SweepBuffer.touches, expected 72 got %v", offset_of(SweepBuffer, touches))
    testing.expectf(t, offset_of(SweepBuffer, maxNbTouches) == 80, "Wrong offset for SweepBuffer.maxNbTouches, expected 80 got %v", offset_of(SweepBuffer, maxNbTouches))
    testing.expectf(t, offset_of(SweepBuffer, nbTouches) == 84, "Wrong offset for SweepBuffer.nbTouches, expected 84 got %v", offset_of(SweepBuffer, nbTouches))
    testing.expectf(t, size_of(SweepBuffer) == 88, "Wrong size for type SweepBuffer, expected 88 got %v", size_of(SweepBuffer))
}

@(test)
test_layout_QueryCache :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(QueryCache, actor) == 8, "Wrong offset for QueryCache.actor, expected 8 got %v", offset_of(QueryCache, actor))
    testing.expectf(t, offset_of(QueryCache, faceIndex) == 16, "Wrong offset for QueryCache.faceIndex, expected 16 got %v", offset_of(QueryCache, faceIndex))
    testing.expectf(t, size_of(QueryCache) == 24, "Wrong size for type QueryCache, expected 24 got %v", size_of(QueryCache))
}

@(test)
test_layout_QueryFilterData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(QueryFilterData, flags) == 16, "Wrong offset for QueryFilterData.flags, expected 16 got %v", offset_of(QueryFilterData, flags))
    testing.expectf(t, size_of(QueryFilterData) == 20, "Wrong size for type QueryFilterData, expected 20 got %v", size_of(QueryFilterData))
}

@(test)
test_layout_QueryFilterCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(QueryFilterCallback) == 8, "Wrong size for type QueryFilterCallback, expected 8 got %v", size_of(QueryFilterCallback))
}

@(test)
test_layout_RigidDynamic :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RigidDynamic) == 24, "Wrong size for type RigidDynamic, expected 24 got %v", size_of(RigidDynamic))
}

@(test)
test_layout_RigidStatic :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RigidStatic) == 24, "Wrong size for type RigidStatic, expected 24 got %v", size_of(RigidStatic))
}

@(test)
test_layout_SceneQueryDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SceneQueryDesc, dynamicStructure) == 4, "Wrong offset for SceneQueryDesc.dynamicStructure, expected 4 got %v", offset_of(SceneQueryDesc, dynamicStructure))
    testing.expectf(t, offset_of(SceneQueryDesc, dynamicTreeRebuildRateHint) == 8, "Wrong offset for SceneQueryDesc.dynamicTreeRebuildRateHint, expected 8 got %v", offset_of(SceneQueryDesc, dynamicTreeRebuildRateHint))
    testing.expectf(t, offset_of(SceneQueryDesc, dynamicTreeSecondaryPruner) == 12, "Wrong offset for SceneQueryDesc.dynamicTreeSecondaryPruner, expected 12 got %v", offset_of(SceneQueryDesc, dynamicTreeSecondaryPruner))
    testing.expectf(t, offset_of(SceneQueryDesc, staticBVHBuildStrategy) == 16, "Wrong offset for SceneQueryDesc.staticBVHBuildStrategy, expected 16 got %v", offset_of(SceneQueryDesc, staticBVHBuildStrategy))
    testing.expectf(t, offset_of(SceneQueryDesc, dynamicBVHBuildStrategy) == 20, "Wrong offset for SceneQueryDesc.dynamicBVHBuildStrategy, expected 20 got %v", offset_of(SceneQueryDesc, dynamicBVHBuildStrategy))
    testing.expectf(t, offset_of(SceneQueryDesc, staticNbObjectsPerNode) == 24, "Wrong offset for SceneQueryDesc.staticNbObjectsPerNode, expected 24 got %v", offset_of(SceneQueryDesc, staticNbObjectsPerNode))
    testing.expectf(t, offset_of(SceneQueryDesc, dynamicNbObjectsPerNode) == 28, "Wrong offset for SceneQueryDesc.dynamicNbObjectsPerNode, expected 28 got %v", offset_of(SceneQueryDesc, dynamicNbObjectsPerNode))
    testing.expectf(t, offset_of(SceneQueryDesc, sceneQueryUpdateMode) == 32, "Wrong offset for SceneQueryDesc.sceneQueryUpdateMode, expected 32 got %v", offset_of(SceneQueryDesc, sceneQueryUpdateMode))
    testing.expectf(t, size_of(SceneQueryDesc) == 36, "Wrong size for type SceneQueryDesc, expected 36 got %v", size_of(SceneQueryDesc))
}

@(test)
test_layout_SceneQuerySystemBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SceneQuerySystemBase) == 8, "Wrong size for type SceneQuerySystemBase, expected 8 got %v", size_of(SceneQuerySystemBase))
}

@(test)
test_layout_SceneSQSystem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SceneSQSystem) == 8, "Wrong size for type SceneSQSystem, expected 8 got %v", size_of(SceneSQSystem))
}

@(test)
test_layout_SceneQuerySystem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SceneQuerySystem) == 8, "Wrong size for type SceneQuerySystem, expected 8 got %v", size_of(SceneQuerySystem))
}

@(test)
test_layout_BroadPhaseRegion :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BroadPhaseRegion, mUserData) == 24, "Wrong offset for BroadPhaseRegion.mUserData, expected 24 got %v", offset_of(BroadPhaseRegion, mUserData))
    testing.expectf(t, size_of(BroadPhaseRegion) == 32, "Wrong size for type BroadPhaseRegion, expected 32 got %v", size_of(BroadPhaseRegion))
}

@(test)
test_layout_BroadPhaseRegionInfo :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BroadPhaseRegionInfo, mNbStaticObjects) == 32, "Wrong offset for BroadPhaseRegionInfo.mNbStaticObjects, expected 32 got %v", offset_of(BroadPhaseRegionInfo, mNbStaticObjects))
    testing.expectf(t, offset_of(BroadPhaseRegionInfo, mNbDynamicObjects) == 36, "Wrong offset for BroadPhaseRegionInfo.mNbDynamicObjects, expected 36 got %v", offset_of(BroadPhaseRegionInfo, mNbDynamicObjects))
    testing.expectf(t, offset_of(BroadPhaseRegionInfo, mActive) == 40, "Wrong offset for BroadPhaseRegionInfo.mActive, expected 40 got %v", offset_of(BroadPhaseRegionInfo, mActive))
    testing.expectf(t, offset_of(BroadPhaseRegionInfo, mOverlap) == 41, "Wrong offset for BroadPhaseRegionInfo.mOverlap, expected 41 got %v", offset_of(BroadPhaseRegionInfo, mOverlap))
    testing.expectf(t, size_of(BroadPhaseRegionInfo) == 48, "Wrong size for type BroadPhaseRegionInfo, expected 48 got %v", size_of(BroadPhaseRegionInfo))
}

@(test)
test_layout_BroadPhaseCaps :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BroadPhaseCaps) == 4, "Wrong size for type BroadPhaseCaps, expected 4 got %v", size_of(BroadPhaseCaps))
}

@(test)
test_layout_BroadPhaseDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BroadPhaseDesc, mContextID) == 8, "Wrong offset for BroadPhaseDesc.mContextID, expected 8 got %v", offset_of(BroadPhaseDesc, mContextID))
    testing.expectf(t, offset_of(BroadPhaseDesc, mFoundLostPairsCapacity) == 24, "Wrong offset for BroadPhaseDesc.mFoundLostPairsCapacity, expected 24 got %v", offset_of(BroadPhaseDesc, mFoundLostPairsCapacity))
    testing.expectf(t, offset_of(BroadPhaseDesc, mDiscardStaticVsKinematic) == 28, "Wrong offset for BroadPhaseDesc.mDiscardStaticVsKinematic, expected 28 got %v", offset_of(BroadPhaseDesc, mDiscardStaticVsKinematic))
    testing.expectf(t, offset_of(BroadPhaseDesc, mDiscardKinematicVsKinematic) == 29, "Wrong offset for BroadPhaseDesc.mDiscardKinematicVsKinematic, expected 29 got %v", offset_of(BroadPhaseDesc, mDiscardKinematicVsKinematic))
    testing.expectf(t, size_of(BroadPhaseDesc) == 32, "Wrong size for type BroadPhaseDesc, expected 32 got %v", size_of(BroadPhaseDesc))
}

@(test)
test_layout_BroadPhaseUpdateData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mNbCreated) == 8, "Wrong offset for BroadPhaseUpdateData.mNbCreated, expected 8 got %v", offset_of(BroadPhaseUpdateData, mNbCreated))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mUpdated) == 16, "Wrong offset for BroadPhaseUpdateData.mUpdated, expected 16 got %v", offset_of(BroadPhaseUpdateData, mUpdated))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mNbUpdated) == 24, "Wrong offset for BroadPhaseUpdateData.mNbUpdated, expected 24 got %v", offset_of(BroadPhaseUpdateData, mNbUpdated))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mRemoved) == 32, "Wrong offset for BroadPhaseUpdateData.mRemoved, expected 32 got %v", offset_of(BroadPhaseUpdateData, mRemoved))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mNbRemoved) == 40, "Wrong offset for BroadPhaseUpdateData.mNbRemoved, expected 40 got %v", offset_of(BroadPhaseUpdateData, mNbRemoved))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mBounds) == 48, "Wrong offset for BroadPhaseUpdateData.mBounds, expected 48 got %v", offset_of(BroadPhaseUpdateData, mBounds))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mGroups) == 56, "Wrong offset for BroadPhaseUpdateData.mGroups, expected 56 got %v", offset_of(BroadPhaseUpdateData, mGroups))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mDistances) == 64, "Wrong offset for BroadPhaseUpdateData.mDistances, expected 64 got %v", offset_of(BroadPhaseUpdateData, mDistances))
    testing.expectf(t, offset_of(BroadPhaseUpdateData, mCapacity) == 72, "Wrong offset for BroadPhaseUpdateData.mCapacity, expected 72 got %v", offset_of(BroadPhaseUpdateData, mCapacity))
    testing.expectf(t, size_of(BroadPhaseUpdateData) == 80, "Wrong size for type BroadPhaseUpdateData, expected 80 got %v", size_of(BroadPhaseUpdateData))
}

@(test)
test_layout_BroadPhasePair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BroadPhasePair, mID1) == 4, "Wrong offset for BroadPhasePair.mID1, expected 4 got %v", offset_of(BroadPhasePair, mID1))
    testing.expectf(t, size_of(BroadPhasePair) == 8, "Wrong size for type BroadPhasePair, expected 8 got %v", size_of(BroadPhasePair))
}

@(test)
test_layout_BroadPhaseResults :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BroadPhaseResults, mCreatedPairs) == 8, "Wrong offset for BroadPhaseResults.mCreatedPairs, expected 8 got %v", offset_of(BroadPhaseResults, mCreatedPairs))
    testing.expectf(t, offset_of(BroadPhaseResults, mNbDeletedPairs) == 16, "Wrong offset for BroadPhaseResults.mNbDeletedPairs, expected 16 got %v", offset_of(BroadPhaseResults, mNbDeletedPairs))
    testing.expectf(t, offset_of(BroadPhaseResults, mDeletedPairs) == 24, "Wrong offset for BroadPhaseResults.mDeletedPairs, expected 24 got %v", offset_of(BroadPhaseResults, mDeletedPairs))
    testing.expectf(t, size_of(BroadPhaseResults) == 32, "Wrong size for type BroadPhaseResults, expected 32 got %v", size_of(BroadPhaseResults))
}

@(test)
test_layout_BroadPhaseRegions :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BroadPhaseRegions) == 8, "Wrong size for type BroadPhaseRegions, expected 8 got %v", size_of(BroadPhaseRegions))
}

@(test)
test_layout_BroadPhase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BroadPhase) == 8, "Wrong size for type BroadPhase, expected 8 got %v", size_of(BroadPhase))
}

@(test)
test_layout_AABBManager :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(AABBManager) == 8, "Wrong size for type AABBManager, expected 8 got %v", size_of(AABBManager))
}

@(test)
test_layout_SceneLimits :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SceneLimits, maxNbBodies) == 4, "Wrong offset for SceneLimits.maxNbBodies, expected 4 got %v", offset_of(SceneLimits, maxNbBodies))
    testing.expectf(t, offset_of(SceneLimits, maxNbStaticShapes) == 8, "Wrong offset for SceneLimits.maxNbStaticShapes, expected 8 got %v", offset_of(SceneLimits, maxNbStaticShapes))
    testing.expectf(t, offset_of(SceneLimits, maxNbDynamicShapes) == 12, "Wrong offset for SceneLimits.maxNbDynamicShapes, expected 12 got %v", offset_of(SceneLimits, maxNbDynamicShapes))
    testing.expectf(t, offset_of(SceneLimits, maxNbAggregates) == 16, "Wrong offset for SceneLimits.maxNbAggregates, expected 16 got %v", offset_of(SceneLimits, maxNbAggregates))
    testing.expectf(t, offset_of(SceneLimits, maxNbConstraints) == 20, "Wrong offset for SceneLimits.maxNbConstraints, expected 20 got %v", offset_of(SceneLimits, maxNbConstraints))
    testing.expectf(t, offset_of(SceneLimits, maxNbRegions) == 24, "Wrong offset for SceneLimits.maxNbRegions, expected 24 got %v", offset_of(SceneLimits, maxNbRegions))
    testing.expectf(t, offset_of(SceneLimits, maxNbBroadPhaseOverlaps) == 28, "Wrong offset for SceneLimits.maxNbBroadPhaseOverlaps, expected 28 got %v", offset_of(SceneLimits, maxNbBroadPhaseOverlaps))
    testing.expectf(t, size_of(SceneLimits) == 32, "Wrong size for type SceneLimits, expected 32 got %v", size_of(SceneLimits))
}

@(test)
test_layout_gDynamicsMemoryConfig :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, maxRigidContactCount) == 4, "Wrong offset for gDynamicsMemoryConfig.maxRigidContactCount, expected 4 got %v", offset_of(gDynamicsMemoryConfig, maxRigidContactCount))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, maxRigidPatchCount) == 8, "Wrong offset for gDynamicsMemoryConfig.maxRigidPatchCount, expected 8 got %v", offset_of(gDynamicsMemoryConfig, maxRigidPatchCount))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, heapCapacity) == 12, "Wrong offset for gDynamicsMemoryConfig.heapCapacity, expected 12 got %v", offset_of(gDynamicsMemoryConfig, heapCapacity))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, foundLostPairsCapacity) == 16, "Wrong offset for gDynamicsMemoryConfig.foundLostPairsCapacity, expected 16 got %v", offset_of(gDynamicsMemoryConfig, foundLostPairsCapacity))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, foundLostAggregatePairsCapacity) == 20, "Wrong offset for gDynamicsMemoryConfig.foundLostAggregatePairsCapacity, expected 20 got %v", offset_of(gDynamicsMemoryConfig, foundLostAggregatePairsCapacity))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, totalAggregatePairsCapacity) == 24, "Wrong offset for gDynamicsMemoryConfig.totalAggregatePairsCapacity, expected 24 got %v", offset_of(gDynamicsMemoryConfig, totalAggregatePairsCapacity))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, maxSoftBodyContacts) == 28, "Wrong offset for gDynamicsMemoryConfig.maxSoftBodyContacts, expected 28 got %v", offset_of(gDynamicsMemoryConfig, maxSoftBodyContacts))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, maxFemClothContacts) == 32, "Wrong offset for gDynamicsMemoryConfig.maxFemClothContacts, expected 32 got %v", offset_of(gDynamicsMemoryConfig, maxFemClothContacts))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, maxParticleContacts) == 36, "Wrong offset for gDynamicsMemoryConfig.maxParticleContacts, expected 36 got %v", offset_of(gDynamicsMemoryConfig, maxParticleContacts))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, collisionStackSize) == 40, "Wrong offset for gDynamicsMemoryConfig.collisionStackSize, expected 40 got %v", offset_of(gDynamicsMemoryConfig, collisionStackSize))
    testing.expectf(t, offset_of(gDynamicsMemoryConfig, maxHairContacts) == 44, "Wrong offset for gDynamicsMemoryConfig.maxHairContacts, expected 44 got %v", offset_of(gDynamicsMemoryConfig, maxHairContacts))
    testing.expectf(t, size_of(gDynamicsMemoryConfig) == 48, "Wrong size for type gDynamicsMemoryConfig, expected 48 got %v", size_of(gDynamicsMemoryConfig))
}

@(test)
test_layout_SceneDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SceneDesc, gravity) == 36, "Wrong offset for SceneDesc.gravity, expected 36 got %v", offset_of(SceneDesc, gravity))
    testing.expectf(t, offset_of(SceneDesc, simulationEventCallback) == 48, "Wrong offset for SceneDesc.simulationEventCallback, expected 48 got %v", offset_of(SceneDesc, simulationEventCallback))
    testing.expectf(t, offset_of(SceneDesc, contactModifyCallback) == 56, "Wrong offset for SceneDesc.contactModifyCallback, expected 56 got %v", offset_of(SceneDesc, contactModifyCallback))
    testing.expectf(t, offset_of(SceneDesc, ccdContactModifyCallback) == 64, "Wrong offset for SceneDesc.ccdContactModifyCallback, expected 64 got %v", offset_of(SceneDesc, ccdContactModifyCallback))
    testing.expectf(t, offset_of(SceneDesc, filterShaderData) == 72, "Wrong offset for SceneDesc.filterShaderData, expected 72 got %v", offset_of(SceneDesc, filterShaderData))
    testing.expectf(t, offset_of(SceneDesc, filterShaderDataSize) == 80, "Wrong offset for SceneDesc.filterShaderDataSize, expected 80 got %v", offset_of(SceneDesc, filterShaderDataSize))
    testing.expectf(t, offset_of(SceneDesc, filterShader) == 88, "Wrong offset for SceneDesc.filterShader, expected 88 got %v", offset_of(SceneDesc, filterShader))
    testing.expectf(t, offset_of(SceneDesc, filterCallback) == 96, "Wrong offset for SceneDesc.filterCallback, expected 96 got %v", offset_of(SceneDesc, filterCallback))
    testing.expectf(t, offset_of(SceneDesc, kineKineFilteringMode) == 104, "Wrong offset for SceneDesc.kineKineFilteringMode, expected 104 got %v", offset_of(SceneDesc, kineKineFilteringMode))
    testing.expectf(t, offset_of(SceneDesc, staticKineFilteringMode) == 108, "Wrong offset for SceneDesc.staticKineFilteringMode, expected 108 got %v", offset_of(SceneDesc, staticKineFilteringMode))
    testing.expectf(t, offset_of(SceneDesc, broadPhaseType) == 112, "Wrong offset for SceneDesc.broadPhaseType, expected 112 got %v", offset_of(SceneDesc, broadPhaseType))
    testing.expectf(t, offset_of(SceneDesc, broadPhaseCallback) == 120, "Wrong offset for SceneDesc.broadPhaseCallback, expected 120 got %v", offset_of(SceneDesc, broadPhaseCallback))
    testing.expectf(t, offset_of(SceneDesc, limits) == 128, "Wrong offset for SceneDesc.limits, expected 128 got %v", offset_of(SceneDesc, limits))
    testing.expectf(t, offset_of(SceneDesc, frictionType) == 160, "Wrong offset for SceneDesc.frictionType, expected 160 got %v", offset_of(SceneDesc, frictionType))
    testing.expectf(t, offset_of(SceneDesc, solverType) == 164, "Wrong offset for SceneDesc.solverType, expected 164 got %v", offset_of(SceneDesc, solverType))
    testing.expectf(t, offset_of(SceneDesc, bounceThresholdVelocity) == 168, "Wrong offset for SceneDesc.bounceThresholdVelocity, expected 168 got %v", offset_of(SceneDesc, bounceThresholdVelocity))
    testing.expectf(t, offset_of(SceneDesc, frictionOffsetThreshold) == 172, "Wrong offset for SceneDesc.frictionOffsetThreshold, expected 172 got %v", offset_of(SceneDesc, frictionOffsetThreshold))
    testing.expectf(t, offset_of(SceneDesc, frictionCorrelationDistance) == 176, "Wrong offset for SceneDesc.frictionCorrelationDistance, expected 176 got %v", offset_of(SceneDesc, frictionCorrelationDistance))
    testing.expectf(t, offset_of(SceneDesc, flags) == 180, "Wrong offset for SceneDesc.flags, expected 180 got %v", offset_of(SceneDesc, flags))
    testing.expectf(t, offset_of(SceneDesc, cpuDispatcher) == 184, "Wrong offset for SceneDesc.cpuDispatcher, expected 184 got %v", offset_of(SceneDesc, cpuDispatcher))
    testing.expectf(t, offset_of(SceneDesc, userData) == 200, "Wrong offset for SceneDesc.userData, expected 200 got %v", offset_of(SceneDesc, userData))
    testing.expectf(t, offset_of(SceneDesc, solverBatchSize) == 208, "Wrong offset for SceneDesc.solverBatchSize, expected 208 got %v", offset_of(SceneDesc, solverBatchSize))
    testing.expectf(t, offset_of(SceneDesc, solverArticulationBatchSize) == 212, "Wrong offset for SceneDesc.solverArticulationBatchSize, expected 212 got %v", offset_of(SceneDesc, solverArticulationBatchSize))
    testing.expectf(t, offset_of(SceneDesc, nbContactDataBlocks) == 216, "Wrong offset for SceneDesc.nbContactDataBlocks, expected 216 got %v", offset_of(SceneDesc, nbContactDataBlocks))
    testing.expectf(t, offset_of(SceneDesc, maxNbContactDataBlocks) == 220, "Wrong offset for SceneDesc.maxNbContactDataBlocks, expected 220 got %v", offset_of(SceneDesc, maxNbContactDataBlocks))
    testing.expectf(t, offset_of(SceneDesc, maxBiasCoefficient) == 224, "Wrong offset for SceneDesc.maxBiasCoefficient, expected 224 got %v", offset_of(SceneDesc, maxBiasCoefficient))
    testing.expectf(t, offset_of(SceneDesc, contactReportStreamBufferSize) == 228, "Wrong offset for SceneDesc.contactReportStreamBufferSize, expected 228 got %v", offset_of(SceneDesc, contactReportStreamBufferSize))
    testing.expectf(t, offset_of(SceneDesc, ccdMaxPasses) == 232, "Wrong offset for SceneDesc.ccdMaxPasses, expected 232 got %v", offset_of(SceneDesc, ccdMaxPasses))
    testing.expectf(t, offset_of(SceneDesc, ccdThreshold) == 236, "Wrong offset for SceneDesc.ccdThreshold, expected 236 got %v", offset_of(SceneDesc, ccdThreshold))
    testing.expectf(t, offset_of(SceneDesc, ccdMaxSeparation) == 240, "Wrong offset for SceneDesc.ccdMaxSeparation, expected 240 got %v", offset_of(SceneDesc, ccdMaxSeparation))
    testing.expectf(t, offset_of(SceneDesc, wakeCounterResetValue) == 244, "Wrong offset for SceneDesc.wakeCounterResetValue, expected 244 got %v", offset_of(SceneDesc, wakeCounterResetValue))
    testing.expectf(t, offset_of(SceneDesc, sanityBounds) == 248, "Wrong offset for SceneDesc.sanityBounds, expected 248 got %v", offset_of(SceneDesc, sanityBounds))
    testing.expectf(t, offset_of(SceneDesc, gpuDynamicsConfig) == 272, "Wrong offset for SceneDesc.gpuDynamicsConfig, expected 272 got %v", offset_of(SceneDesc, gpuDynamicsConfig))
    testing.expectf(t, offset_of(SceneDesc, gpuMaxNumPartitions) == 320, "Wrong offset for SceneDesc.gpuMaxNumPartitions, expected 320 got %v", offset_of(SceneDesc, gpuMaxNumPartitions))
    testing.expectf(t, offset_of(SceneDesc, gpuMaxNumStaticPartitions) == 324, "Wrong offset for SceneDesc.gpuMaxNumStaticPartitions, expected 324 got %v", offset_of(SceneDesc, gpuMaxNumStaticPartitions))
    testing.expectf(t, offset_of(SceneDesc, gpuComputeVersion) == 328, "Wrong offset for SceneDesc.gpuComputeVersion, expected 328 got %v", offset_of(SceneDesc, gpuComputeVersion))
    testing.expectf(t, offset_of(SceneDesc, contactPairSlabSize) == 332, "Wrong offset for SceneDesc.contactPairSlabSize, expected 332 got %v", offset_of(SceneDesc, contactPairSlabSize))
    testing.expectf(t, offset_of(SceneDesc, sceneQuerySystem) == 336, "Wrong offset for SceneDesc.sceneQuerySystem, expected 336 got %v", offset_of(SceneDesc, sceneQuerySystem))
    testing.expectf(t, size_of(SceneDesc) == 352, "Wrong size for type SceneDesc, expected 352 got %v", size_of(SceneDesc))
}

@(test)
test_layout_SimulationStatistics :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SimulationStatistics, nbActiveDynamicBodies) == 4, "Wrong offset for SimulationStatistics.nbActiveDynamicBodies, expected 4 got %v", offset_of(SimulationStatistics, nbActiveDynamicBodies))
    testing.expectf(t, offset_of(SimulationStatistics, nbActiveKinematicBodies) == 8, "Wrong offset for SimulationStatistics.nbActiveKinematicBodies, expected 8 got %v", offset_of(SimulationStatistics, nbActiveKinematicBodies))
    testing.expectf(t, offset_of(SimulationStatistics, nbStaticBodies) == 12, "Wrong offset for SimulationStatistics.nbStaticBodies, expected 12 got %v", offset_of(SimulationStatistics, nbStaticBodies))
    testing.expectf(t, offset_of(SimulationStatistics, nbDynamicBodies) == 16, "Wrong offset for SimulationStatistics.nbDynamicBodies, expected 16 got %v", offset_of(SimulationStatistics, nbDynamicBodies))
    testing.expectf(t, offset_of(SimulationStatistics, nbKinematicBodies) == 20, "Wrong offset for SimulationStatistics.nbKinematicBodies, expected 20 got %v", offset_of(SimulationStatistics, nbKinematicBodies))
    testing.expectf(t, offset_of(SimulationStatistics, nbAggregates) == 68, "Wrong offset for SimulationStatistics.nbAggregates, expected 68 got %v", offset_of(SimulationStatistics, nbAggregates))
    testing.expectf(t, offset_of(SimulationStatistics, nbArticulations) == 72, "Wrong offset for SimulationStatistics.nbArticulations, expected 72 got %v", offset_of(SimulationStatistics, nbArticulations))
    testing.expectf(t, offset_of(SimulationStatistics, nbAxisSolverConstraints) == 76, "Wrong offset for SimulationStatistics.nbAxisSolverConstraints, expected 76 got %v", offset_of(SimulationStatistics, nbAxisSolverConstraints))
    testing.expectf(t, offset_of(SimulationStatistics, compressedContactSize) == 80, "Wrong offset for SimulationStatistics.compressedContactSize, expected 80 got %v", offset_of(SimulationStatistics, compressedContactSize))
    testing.expectf(t, offset_of(SimulationStatistics, requiredContactConstraintMemory) == 84, "Wrong offset for SimulationStatistics.requiredContactConstraintMemory, expected 84 got %v", offset_of(SimulationStatistics, requiredContactConstraintMemory))
    testing.expectf(t, offset_of(SimulationStatistics, peakConstraintMemory) == 88, "Wrong offset for SimulationStatistics.peakConstraintMemory, expected 88 got %v", offset_of(SimulationStatistics, peakConstraintMemory))
    testing.expectf(t, offset_of(SimulationStatistics, nbDiscreteContactPairsTotal) == 92, "Wrong offset for SimulationStatistics.nbDiscreteContactPairsTotal, expected 92 got %v", offset_of(SimulationStatistics, nbDiscreteContactPairsTotal))
    testing.expectf(t, offset_of(SimulationStatistics, nbDiscreteContactPairsWithCacheHits) == 96, "Wrong offset for SimulationStatistics.nbDiscreteContactPairsWithCacheHits, expected 96 got %v", offset_of(SimulationStatistics, nbDiscreteContactPairsWithCacheHits))
    testing.expectf(t, offset_of(SimulationStatistics, nbDiscreteContactPairsWithContacts) == 100, "Wrong offset for SimulationStatistics.nbDiscreteContactPairsWithContacts, expected 100 got %v", offset_of(SimulationStatistics, nbDiscreteContactPairsWithContacts))
    testing.expectf(t, offset_of(SimulationStatistics, nbNewPairs) == 104, "Wrong offset for SimulationStatistics.nbNewPairs, expected 104 got %v", offset_of(SimulationStatistics, nbNewPairs))
    testing.expectf(t, offset_of(SimulationStatistics, nbLostPairs) == 108, "Wrong offset for SimulationStatistics.nbLostPairs, expected 108 got %v", offset_of(SimulationStatistics, nbLostPairs))
    testing.expectf(t, offset_of(SimulationStatistics, nbNewTouches) == 112, "Wrong offset for SimulationStatistics.nbNewTouches, expected 112 got %v", offset_of(SimulationStatistics, nbNewTouches))
    testing.expectf(t, offset_of(SimulationStatistics, nbLostTouches) == 116, "Wrong offset for SimulationStatistics.nbLostTouches, expected 116 got %v", offset_of(SimulationStatistics, nbLostTouches))
    testing.expectf(t, offset_of(SimulationStatistics, nbPartitions) == 120, "Wrong offset for SimulationStatistics.nbPartitions, expected 120 got %v", offset_of(SimulationStatistics, nbPartitions))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemParticles) == 128, "Wrong offset for SimulationStatistics.gpuMemParticles, expected 128 got %v", offset_of(SimulationStatistics, gpuMemParticles))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemSoftBodies) == 136, "Wrong offset for SimulationStatistics.gpuMemSoftBodies, expected 136 got %v", offset_of(SimulationStatistics, gpuMemSoftBodies))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemFEMCloths) == 144, "Wrong offset for SimulationStatistics.gpuMemFEMCloths, expected 144 got %v", offset_of(SimulationStatistics, gpuMemFEMCloths))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHairSystems) == 152, "Wrong offset for SimulationStatistics.gpuMemHairSystems, expected 152 got %v", offset_of(SimulationStatistics, gpuMemHairSystems))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeap) == 160, "Wrong offset for SimulationStatistics.gpuMemHeap, expected 160 got %v", offset_of(SimulationStatistics, gpuMemHeap))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapBroadPhase) == 168, "Wrong offset for SimulationStatistics.gpuMemHeapBroadPhase, expected 168 got %v", offset_of(SimulationStatistics, gpuMemHeapBroadPhase))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapNarrowPhase) == 176, "Wrong offset for SimulationStatistics.gpuMemHeapNarrowPhase, expected 176 got %v", offset_of(SimulationStatistics, gpuMemHeapNarrowPhase))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSolver) == 184, "Wrong offset for SimulationStatistics.gpuMemHeapSolver, expected 184 got %v", offset_of(SimulationStatistics, gpuMemHeapSolver))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapArticulation) == 192, "Wrong offset for SimulationStatistics.gpuMemHeapArticulation, expected 192 got %v", offset_of(SimulationStatistics, gpuMemHeapArticulation))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSimulation) == 200, "Wrong offset for SimulationStatistics.gpuMemHeapSimulation, expected 200 got %v", offset_of(SimulationStatistics, gpuMemHeapSimulation))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSimulationArticulation) == 208, "Wrong offset for SimulationStatistics.gpuMemHeapSimulationArticulation, expected 208 got %v", offset_of(SimulationStatistics, gpuMemHeapSimulationArticulation))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSimulationParticles) == 216, "Wrong offset for SimulationStatistics.gpuMemHeapSimulationParticles, expected 216 got %v", offset_of(SimulationStatistics, gpuMemHeapSimulationParticles))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSimulationSoftBody) == 224, "Wrong offset for SimulationStatistics.gpuMemHeapSimulationSoftBody, expected 224 got %v", offset_of(SimulationStatistics, gpuMemHeapSimulationSoftBody))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSimulationFEMCloth) == 232, "Wrong offset for SimulationStatistics.gpuMemHeapSimulationFEMCloth, expected 232 got %v", offset_of(SimulationStatistics, gpuMemHeapSimulationFEMCloth))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSimulationHairSystem) == 240, "Wrong offset for SimulationStatistics.gpuMemHeapSimulationHairSystem, expected 240 got %v", offset_of(SimulationStatistics, gpuMemHeapSimulationHairSystem))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapParticles) == 248, "Wrong offset for SimulationStatistics.gpuMemHeapParticles, expected 248 got %v", offset_of(SimulationStatistics, gpuMemHeapParticles))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapSoftBodies) == 256, "Wrong offset for SimulationStatistics.gpuMemHeapSoftBodies, expected 256 got %v", offset_of(SimulationStatistics, gpuMemHeapSoftBodies))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapFEMCloths) == 264, "Wrong offset for SimulationStatistics.gpuMemHeapFEMCloths, expected 264 got %v", offset_of(SimulationStatistics, gpuMemHeapFEMCloths))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapHairSystems) == 272, "Wrong offset for SimulationStatistics.gpuMemHeapHairSystems, expected 272 got %v", offset_of(SimulationStatistics, gpuMemHeapHairSystems))
    testing.expectf(t, offset_of(SimulationStatistics, gpuMemHeapOther) == 280, "Wrong offset for SimulationStatistics.gpuMemHeapOther, expected 280 got %v", offset_of(SimulationStatistics, gpuMemHeapOther))
    testing.expectf(t, offset_of(SimulationStatistics, nbBroadPhaseAdds) == 288, "Wrong offset for SimulationStatistics.nbBroadPhaseAdds, expected 288 got %v", offset_of(SimulationStatistics, nbBroadPhaseAdds))
    testing.expectf(t, offset_of(SimulationStatistics, nbBroadPhaseRemoves) == 292, "Wrong offset for SimulationStatistics.nbBroadPhaseRemoves, expected 292 got %v", offset_of(SimulationStatistics, nbBroadPhaseRemoves))
    testing.expectf(t, size_of(SimulationStatistics) == 2232, "Wrong size for type SimulationStatistics, expected 2232 got %v", size_of(SimulationStatistics))
}

@(test)
test_layout_GpuBodyData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GpuBodyData, pos) == 16, "Wrong offset for GpuBodyData.pos, expected 16 got %v", offset_of(GpuBodyData, pos))
    testing.expectf(t, offset_of(GpuBodyData, linVel) == 32, "Wrong offset for GpuBodyData.linVel, expected 32 got %v", offset_of(GpuBodyData, linVel))
    testing.expectf(t, offset_of(GpuBodyData, angVel) == 48, "Wrong offset for GpuBodyData.angVel, expected 48 got %v", offset_of(GpuBodyData, angVel))
    testing.expectf(t, size_of(GpuBodyData) == 64, "Wrong size for type GpuBodyData, expected 64 got %v", size_of(GpuBodyData))
}

@(test)
test_layout_GpuActorPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GpuActorPair, nodeIndex) == 8, "Wrong offset for GpuActorPair.nodeIndex, expected 8 got %v", offset_of(GpuActorPair, nodeIndex))
    testing.expectf(t, size_of(GpuActorPair) == 16, "Wrong size for type GpuActorPair, expected 16 got %v", size_of(GpuActorPair))
}

@(test)
test_layout_IndexDataPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(IndexDataPair, data) == 8, "Wrong offset for IndexDataPair.data, expected 8 got %v", offset_of(IndexDataPair, data))
    testing.expectf(t, size_of(IndexDataPair) == 16, "Wrong size for type IndexDataPair, expected 16 got %v", size_of(IndexDataPair))
}

@(test)
test_layout_PvdSceneClient :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PvdSceneClient) == 8, "Wrong size for type PvdSceneClient, expected 8 got %v", size_of(PvdSceneClient))
}

@(test)
test_layout_DominanceGroupPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(DominanceGroupPair, dominance1) == 1, "Wrong offset for DominanceGroupPair.dominance1, expected 1 got %v", offset_of(DominanceGroupPair, dominance1))
    testing.expectf(t, size_of(DominanceGroupPair) == 2, "Wrong size for type DominanceGroupPair, expected 2 got %v", size_of(DominanceGroupPair))
}

@(test)
test_layout_BroadPhaseCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BroadPhaseCallback) == 8, "Wrong size for type BroadPhaseCallback, expected 8 got %v", size_of(BroadPhaseCallback))
}

@(test)
test_layout_Scene :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Scene, userData) == 8, "Wrong offset for Scene.userData, expected 8 got %v", offset_of(Scene, userData))
    testing.expectf(t, size_of(Scene) == 16, "Wrong size for type Scene, expected 16 got %v", size_of(Scene))
}

@(test)
test_layout_SceneReadLock :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SceneReadLock) == 8, "Wrong size for type SceneReadLock, expected 8 got %v", size_of(SceneReadLock))
}

@(test)
test_layout_SceneWriteLock :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SceneWriteLock) == 8, "Wrong size for type SceneWriteLock, expected 8 got %v", size_of(SceneWriteLock))
}

@(test)
test_layout_ContactPairExtraDataItem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ContactPairExtraDataItem) == 1, "Wrong size for type ContactPairExtraDataItem, expected 1 got %v", size_of(ContactPairExtraDataItem))
}

@(test)
test_layout_ContactPairVelocity :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ContactPairVelocity) == 52, "Wrong size for type ContactPairVelocity, expected 52 got %v", size_of(ContactPairVelocity))
}

@(test)
test_layout_ContactPairPose :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ContactPairPose) == 60, "Wrong size for type ContactPairPose, expected 60 got %v", size_of(ContactPairPose))
}

@(test)
test_layout_ContactPairIndex :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPairIndex, index) == 2, "Wrong offset for ContactPairIndex.index, expected 2 got %v", offset_of(ContactPairIndex, index))
    testing.expectf(t, size_of(ContactPairIndex) == 4, "Wrong size for type ContactPairIndex, expected 4 got %v", size_of(ContactPairIndex))
}

@(test)
test_layout_ContactPairExtraDataIterator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPairExtraDataIterator, endPtr) == 8, "Wrong offset for ContactPairExtraDataIterator.endPtr, expected 8 got %v", offset_of(ContactPairExtraDataIterator, endPtr))
    testing.expectf(t, offset_of(ContactPairExtraDataIterator, preSolverVelocity) == 16, "Wrong offset for ContactPairExtraDataIterator.preSolverVelocity, expected 16 got %v", offset_of(ContactPairExtraDataIterator, preSolverVelocity))
    testing.expectf(t, offset_of(ContactPairExtraDataIterator, postSolverVelocity) == 24, "Wrong offset for ContactPairExtraDataIterator.postSolverVelocity, expected 24 got %v", offset_of(ContactPairExtraDataIterator, postSolverVelocity))
    testing.expectf(t, offset_of(ContactPairExtraDataIterator, eventPose) == 32, "Wrong offset for ContactPairExtraDataIterator.eventPose, expected 32 got %v", offset_of(ContactPairExtraDataIterator, eventPose))
    testing.expectf(t, offset_of(ContactPairExtraDataIterator, contactPairIndex) == 40, "Wrong offset for ContactPairExtraDataIterator.contactPairIndex, expected 40 got %v", offset_of(ContactPairExtraDataIterator, contactPairIndex))
    testing.expectf(t, size_of(ContactPairExtraDataIterator) == 48, "Wrong size for type ContactPairExtraDataIterator, expected 48 got %v", size_of(ContactPairExtraDataIterator))
}

@(test)
test_layout_ContactPairHeader :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPairHeader, extraDataStream) == 16, "Wrong offset for ContactPairHeader.extraDataStream, expected 16 got %v", offset_of(ContactPairHeader, extraDataStream))
    testing.expectf(t, offset_of(ContactPairHeader, extraDataStreamSize) == 24, "Wrong offset for ContactPairHeader.extraDataStreamSize, expected 24 got %v", offset_of(ContactPairHeader, extraDataStreamSize))
    testing.expectf(t, offset_of(ContactPairHeader, flags) == 26, "Wrong offset for ContactPairHeader.flags, expected 26 got %v", offset_of(ContactPairHeader, flags))
    testing.expectf(t, offset_of(ContactPairHeader, pairs) == 32, "Wrong offset for ContactPairHeader.pairs, expected 32 got %v", offset_of(ContactPairHeader, pairs))
    testing.expectf(t, offset_of(ContactPairHeader, nbPairs) == 40, "Wrong offset for ContactPairHeader.nbPairs, expected 40 got %v", offset_of(ContactPairHeader, nbPairs))
    testing.expectf(t, size_of(ContactPairHeader) == 48, "Wrong size for type ContactPairHeader, expected 48 got %v", size_of(ContactPairHeader))
}

@(test)
test_layout_ContactPairPoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPairPoint, separation) == 12, "Wrong offset for ContactPairPoint.separation, expected 12 got %v", offset_of(ContactPairPoint, separation))
    testing.expectf(t, offset_of(ContactPairPoint, normal) == 16, "Wrong offset for ContactPairPoint.normal, expected 16 got %v", offset_of(ContactPairPoint, normal))
    testing.expectf(t, offset_of(ContactPairPoint, internalFaceIndex0) == 28, "Wrong offset for ContactPairPoint.internalFaceIndex0, expected 28 got %v", offset_of(ContactPairPoint, internalFaceIndex0))
    testing.expectf(t, offset_of(ContactPairPoint, impulse) == 32, "Wrong offset for ContactPairPoint.impulse, expected 32 got %v", offset_of(ContactPairPoint, impulse))
    testing.expectf(t, offset_of(ContactPairPoint, internalFaceIndex1) == 44, "Wrong offset for ContactPairPoint.internalFaceIndex1, expected 44 got %v", offset_of(ContactPairPoint, internalFaceIndex1))
    testing.expectf(t, size_of(ContactPairPoint) == 48, "Wrong size for type ContactPairPoint, expected 48 got %v", size_of(ContactPairPoint))
}

@(test)
test_layout_ContactPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ContactPair, contactPatches) == 16, "Wrong offset for ContactPair.contactPatches, expected 16 got %v", offset_of(ContactPair, contactPatches))
    testing.expectf(t, offset_of(ContactPair, contactPoints) == 24, "Wrong offset for ContactPair.contactPoints, expected 24 got %v", offset_of(ContactPair, contactPoints))
    testing.expectf(t, offset_of(ContactPair, contactImpulses) == 32, "Wrong offset for ContactPair.contactImpulses, expected 32 got %v", offset_of(ContactPair, contactImpulses))
    testing.expectf(t, offset_of(ContactPair, requiredBufferSize) == 40, "Wrong offset for ContactPair.requiredBufferSize, expected 40 got %v", offset_of(ContactPair, requiredBufferSize))
    testing.expectf(t, offset_of(ContactPair, contactCount) == 44, "Wrong offset for ContactPair.contactCount, expected 44 got %v", offset_of(ContactPair, contactCount))
    testing.expectf(t, offset_of(ContactPair, patchCount) == 45, "Wrong offset for ContactPair.patchCount, expected 45 got %v", offset_of(ContactPair, patchCount))
    testing.expectf(t, offset_of(ContactPair, contactStreamSize) == 46, "Wrong offset for ContactPair.contactStreamSize, expected 46 got %v", offset_of(ContactPair, contactStreamSize))
    testing.expectf(t, offset_of(ContactPair, flags) == 48, "Wrong offset for ContactPair.flags, expected 48 got %v", offset_of(ContactPair, flags))
    testing.expectf(t, offset_of(ContactPair, events) == 50, "Wrong offset for ContactPair.events, expected 50 got %v", offset_of(ContactPair, events))
    testing.expectf(t, size_of(ContactPair) == 64, "Wrong size for type ContactPair, expected 64 got %v", size_of(ContactPair))
}

@(test)
test_layout_TriggerPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TriggerPair, triggerActor) == 8, "Wrong offset for TriggerPair.triggerActor, expected 8 got %v", offset_of(TriggerPair, triggerActor))
    testing.expectf(t, offset_of(TriggerPair, otherShape) == 16, "Wrong offset for TriggerPair.otherShape, expected 16 got %v", offset_of(TriggerPair, otherShape))
    testing.expectf(t, offset_of(TriggerPair, otherActor) == 24, "Wrong offset for TriggerPair.otherActor, expected 24 got %v", offset_of(TriggerPair, otherActor))
    testing.expectf(t, offset_of(TriggerPair, status) == 32, "Wrong offset for TriggerPair.status, expected 32 got %v", offset_of(TriggerPair, status))
    testing.expectf(t, offset_of(TriggerPair, flags) == 36, "Wrong offset for TriggerPair.flags, expected 36 got %v", offset_of(TriggerPair, flags))
    testing.expectf(t, size_of(TriggerPair) == 40, "Wrong size for type TriggerPair, expected 40 got %v", size_of(TriggerPair))
}

@(test)
test_layout_ConstraintInfo :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConstraintInfo, externalReference) == 8, "Wrong offset for ConstraintInfo.externalReference, expected 8 got %v", offset_of(ConstraintInfo, externalReference))
    testing.expectf(t, offset_of(ConstraintInfo, type) == 16, "Wrong offset for ConstraintInfo.type, expected 16 got %v", offset_of(ConstraintInfo, type))
    testing.expectf(t, size_of(ConstraintInfo) == 24, "Wrong size for type ConstraintInfo, expected 24 got %v", size_of(ConstraintInfo))
}

@(test)
test_layout_SimulationEventCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SimulationEventCallback) == 8, "Wrong size for type SimulationEventCallback, expected 8 got %v", size_of(SimulationEventCallback))
}

@(test)
test_layout_FEMParameters :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(FEMParameters, settlingThreshold) == 4, "Wrong offset for FEMParameters.settlingThreshold, expected 4 got %v", offset_of(FEMParameters, settlingThreshold))
    testing.expectf(t, offset_of(FEMParameters, sleepThreshold) == 8, "Wrong offset for FEMParameters.sleepThreshold, expected 8 got %v", offset_of(FEMParameters, sleepThreshold))
    testing.expectf(t, offset_of(FEMParameters, sleepDamping) == 12, "Wrong offset for FEMParameters.sleepDamping, expected 12 got %v", offset_of(FEMParameters, sleepDamping))
    testing.expectf(t, offset_of(FEMParameters, selfCollisionFilterDistance) == 16, "Wrong offset for FEMParameters.selfCollisionFilterDistance, expected 16 got %v", offset_of(FEMParameters, selfCollisionFilterDistance))
    testing.expectf(t, offset_of(FEMParameters, selfCollisionStressTolerance) == 20, "Wrong offset for FEMParameters.selfCollisionStressTolerance, expected 20 got %v", offset_of(FEMParameters, selfCollisionStressTolerance))
    testing.expectf(t, size_of(FEMParameters) == 24, "Wrong size for type FEMParameters, expected 24 got %v", size_of(FEMParameters))
}

@(test)
test_layout_PruningStructure :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PruningStructure) == 16, "Wrong size for type PruningStructure, expected 16 got %v", size_of(PruningStructure))
}

@(test)
test_layout_ExtendedVec3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ExtendedVec3, y) == 8, "Wrong offset for ExtendedVec3.y, expected 8 got %v", offset_of(ExtendedVec3, y))
    testing.expectf(t, offset_of(ExtendedVec3, z) == 16, "Wrong offset for ExtendedVec3.z, expected 16 got %v", offset_of(ExtendedVec3, z))
    testing.expectf(t, size_of(ExtendedVec3) == 24, "Wrong size for type ExtendedVec3, expected 24 got %v", size_of(ExtendedVec3))
}

@(test)
test_layout_Obstacle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Obstacle, mUserData) == 8, "Wrong offset for Obstacle.mUserData, expected 8 got %v", offset_of(Obstacle, mUserData))
    testing.expectf(t, offset_of(Obstacle, mPos) == 16, "Wrong offset for Obstacle.mPos, expected 16 got %v", offset_of(Obstacle, mPos))
    testing.expectf(t, offset_of(Obstacle, mRot) == 40, "Wrong offset for Obstacle.mRot, expected 40 got %v", offset_of(Obstacle, mRot))
    testing.expectf(t, size_of(Obstacle) == 56, "Wrong size for type Obstacle, expected 56 got %v", size_of(Obstacle))
}

@(test)
test_layout_BoxObstacle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BoxObstacle, mHalfExtents) == 56, "Wrong offset for BoxObstacle.mHalfExtents, expected 56 got %v", offset_of(BoxObstacle, mHalfExtents))
    testing.expectf(t, size_of(BoxObstacle) == 72, "Wrong size for type BoxObstacle, expected 72 got %v", size_of(BoxObstacle))
}

@(test)
test_layout_CapsuleObstacle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(CapsuleObstacle, mHalfHeight) == 56, "Wrong offset for CapsuleObstacle.mHalfHeight, expected 56 got %v", offset_of(CapsuleObstacle, mHalfHeight))
    testing.expectf(t, offset_of(CapsuleObstacle, mRadius) == 60, "Wrong offset for CapsuleObstacle.mRadius, expected 60 got %v", offset_of(CapsuleObstacle, mRadius))
    testing.expectf(t, size_of(CapsuleObstacle) == 64, "Wrong size for type CapsuleObstacle, expected 64 got %v", size_of(CapsuleObstacle))
}

@(test)
test_layout_ObstacleContext :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ObstacleContext) == 8, "Wrong size for type ObstacleContext, expected 8 got %v", size_of(ObstacleContext))
}

@(test)
test_layout_ControllerState :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerState, touchedShape) == 16, "Wrong offset for ControllerState.touchedShape, expected 16 got %v", offset_of(ControllerState, touchedShape))
    testing.expectf(t, offset_of(ControllerState, touchedActor) == 24, "Wrong offset for ControllerState.touchedActor, expected 24 got %v", offset_of(ControllerState, touchedActor))
    testing.expectf(t, offset_of(ControllerState, touchedObstacleHandle) == 32, "Wrong offset for ControllerState.touchedObstacleHandle, expected 32 got %v", offset_of(ControllerState, touchedObstacleHandle))
    testing.expectf(t, offset_of(ControllerState, collisionFlags) == 36, "Wrong offset for ControllerState.collisionFlags, expected 36 got %v", offset_of(ControllerState, collisionFlags))
    testing.expectf(t, offset_of(ControllerState, standOnAnotherCCT) == 40, "Wrong offset for ControllerState.standOnAnotherCCT, expected 40 got %v", offset_of(ControllerState, standOnAnotherCCT))
    testing.expectf(t, offset_of(ControllerState, standOnObstacle) == 41, "Wrong offset for ControllerState.standOnObstacle, expected 41 got %v", offset_of(ControllerState, standOnObstacle))
    testing.expectf(t, offset_of(ControllerState, isMovingUp) == 42, "Wrong offset for ControllerState.isMovingUp, expected 42 got %v", offset_of(ControllerState, isMovingUp))
    testing.expectf(t, size_of(ControllerState) == 48, "Wrong size for type ControllerState, expected 48 got %v", size_of(ControllerState))
}

@(test)
test_layout_ControllerStats :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerStats, nbFullUpdates) == 2, "Wrong offset for ControllerStats.nbFullUpdates, expected 2 got %v", offset_of(ControllerStats, nbFullUpdates))
    testing.expectf(t, offset_of(ControllerStats, nbPartialUpdates) == 4, "Wrong offset for ControllerStats.nbPartialUpdates, expected 4 got %v", offset_of(ControllerStats, nbPartialUpdates))
    testing.expectf(t, offset_of(ControllerStats, nbTessellation) == 6, "Wrong offset for ControllerStats.nbTessellation, expected 6 got %v", offset_of(ControllerStats, nbTessellation))
    testing.expectf(t, size_of(ControllerStats) == 8, "Wrong size for type ControllerStats, expected 8 got %v", size_of(ControllerStats))
}

@(test)
test_layout_ControllerHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerHit, worldPos) == 8, "Wrong offset for ControllerHit.worldPos, expected 8 got %v", offset_of(ControllerHit, worldPos))
    testing.expectf(t, offset_of(ControllerHit, worldNormal) == 32, "Wrong offset for ControllerHit.worldNormal, expected 32 got %v", offset_of(ControllerHit, worldNormal))
    testing.expectf(t, offset_of(ControllerHit, dir) == 44, "Wrong offset for ControllerHit.dir, expected 44 got %v", offset_of(ControllerHit, dir))
    testing.expectf(t, offset_of(ControllerHit, length) == 56, "Wrong offset for ControllerHit.length, expected 56 got %v", offset_of(ControllerHit, length))
    testing.expectf(t, size_of(ControllerHit) == 64, "Wrong size for type ControllerHit, expected 64 got %v", size_of(ControllerHit))
}

@(test)
test_layout_ControllerShapeHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerShapeHit, shape) == 64, "Wrong offset for ControllerShapeHit.shape, expected 64 got %v", offset_of(ControllerShapeHit, shape))
    testing.expectf(t, offset_of(ControllerShapeHit, actor) == 72, "Wrong offset for ControllerShapeHit.actor, expected 72 got %v", offset_of(ControllerShapeHit, actor))
    testing.expectf(t, offset_of(ControllerShapeHit, triangleIndex) == 80, "Wrong offset for ControllerShapeHit.triangleIndex, expected 80 got %v", offset_of(ControllerShapeHit, triangleIndex))
    testing.expectf(t, size_of(ControllerShapeHit) == 88, "Wrong size for type ControllerShapeHit, expected 88 got %v", size_of(ControllerShapeHit))
}

@(test)
test_layout_ControllersHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllersHit, other) == 64, "Wrong offset for ControllersHit.other, expected 64 got %v", offset_of(ControllersHit, other))
    testing.expectf(t, size_of(ControllersHit) == 72, "Wrong size for type ControllersHit, expected 72 got %v", size_of(ControllersHit))
}

@(test)
test_layout_ControllerObstacleHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerObstacleHit, userData) == 64, "Wrong offset for ControllerObstacleHit.userData, expected 64 got %v", offset_of(ControllerObstacleHit, userData))
    testing.expectf(t, size_of(ControllerObstacleHit) == 72, "Wrong size for type ControllerObstacleHit, expected 72 got %v", size_of(ControllerObstacleHit))
}

@(test)
test_layout_UserControllerHitReport :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(UserControllerHitReport) == 8, "Wrong size for type UserControllerHitReport, expected 8 got %v", size_of(UserControllerHitReport))
}

@(test)
test_layout_ControllerFilterCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ControllerFilterCallback) == 8, "Wrong size for type ControllerFilterCallback, expected 8 got %v", size_of(ControllerFilterCallback))
}

@(test)
test_layout_ControllerFilters :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerFilters, mFilterCallback) == 8, "Wrong offset for ControllerFilters.mFilterCallback, expected 8 got %v", offset_of(ControllerFilters, mFilterCallback))
    testing.expectf(t, offset_of(ControllerFilters, mFilterFlags) == 16, "Wrong offset for ControllerFilters.mFilterFlags, expected 16 got %v", offset_of(ControllerFilters, mFilterFlags))
    testing.expectf(t, offset_of(ControllerFilters, mCCTFilterCallback) == 24, "Wrong offset for ControllerFilters.mCCTFilterCallback, expected 24 got %v", offset_of(ControllerFilters, mCCTFilterCallback))
    testing.expectf(t, size_of(ControllerFilters) == 32, "Wrong size for type ControllerFilters, expected 32 got %v", size_of(ControllerFilters))
}

@(test)
test_layout_ControllerDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ControllerDesc, position) == 8, "Wrong offset for ControllerDesc.position, expected 8 got %v", offset_of(ControllerDesc, position))
    testing.expectf(t, offset_of(ControllerDesc, upDirection) == 32, "Wrong offset for ControllerDesc.upDirection, expected 32 got %v", offset_of(ControllerDesc, upDirection))
    testing.expectf(t, offset_of(ControllerDesc, slopeLimit) == 44, "Wrong offset for ControllerDesc.slopeLimit, expected 44 got %v", offset_of(ControllerDesc, slopeLimit))
    testing.expectf(t, offset_of(ControllerDesc, invisibleWallHeight) == 48, "Wrong offset for ControllerDesc.invisibleWallHeight, expected 48 got %v", offset_of(ControllerDesc, invisibleWallHeight))
    testing.expectf(t, offset_of(ControllerDesc, maxJumpHeight) == 52, "Wrong offset for ControllerDesc.maxJumpHeight, expected 52 got %v", offset_of(ControllerDesc, maxJumpHeight))
    testing.expectf(t, offset_of(ControllerDesc, contactOffset) == 56, "Wrong offset for ControllerDesc.contactOffset, expected 56 got %v", offset_of(ControllerDesc, contactOffset))
    testing.expectf(t, offset_of(ControllerDesc, stepOffset) == 60, "Wrong offset for ControllerDesc.stepOffset, expected 60 got %v", offset_of(ControllerDesc, stepOffset))
    testing.expectf(t, offset_of(ControllerDesc, density) == 64, "Wrong offset for ControllerDesc.density, expected 64 got %v", offset_of(ControllerDesc, density))
    testing.expectf(t, offset_of(ControllerDesc, scaleCoeff) == 68, "Wrong offset for ControllerDesc.scaleCoeff, expected 68 got %v", offset_of(ControllerDesc, scaleCoeff))
    testing.expectf(t, offset_of(ControllerDesc, volumeGrowth) == 72, "Wrong offset for ControllerDesc.volumeGrowth, expected 72 got %v", offset_of(ControllerDesc, volumeGrowth))
    testing.expectf(t, offset_of(ControllerDesc, reportCallback) == 80, "Wrong offset for ControllerDesc.reportCallback, expected 80 got %v", offset_of(ControllerDesc, reportCallback))
    testing.expectf(t, offset_of(ControllerDesc, behaviorCallback) == 88, "Wrong offset for ControllerDesc.behaviorCallback, expected 88 got %v", offset_of(ControllerDesc, behaviorCallback))
    testing.expectf(t, offset_of(ControllerDesc, nonWalkableMode) == 96, "Wrong offset for ControllerDesc.nonWalkableMode, expected 96 got %v", offset_of(ControllerDesc, nonWalkableMode))
    testing.expectf(t, offset_of(ControllerDesc, material) == 104, "Wrong offset for ControllerDesc.material, expected 104 got %v", offset_of(ControllerDesc, material))
    testing.expectf(t, offset_of(ControllerDesc, registerDeletionListener) == 112, "Wrong offset for ControllerDesc.registerDeletionListener, expected 112 got %v", offset_of(ControllerDesc, registerDeletionListener))
    testing.expectf(t, offset_of(ControllerDesc, clientID) == 113, "Wrong offset for ControllerDesc.clientID, expected 113 got %v", offset_of(ControllerDesc, clientID))
    testing.expectf(t, offset_of(ControllerDesc, userData) == 120, "Wrong offset for ControllerDesc.userData, expected 120 got %v", offset_of(ControllerDesc, userData))
    testing.expectf(t, size_of(ControllerDesc) == 132, "Wrong size for type ControllerDesc, expected 132 got %v", size_of(ControllerDesc))
}

@(test)
test_layout_Controller :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Controller) == 8, "Wrong size for type Controller, expected 8 got %v", size_of(Controller))
}

@(test)
test_layout_BoxControllerDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BoxControllerDesc, halfHeight) == 136, "Wrong offset for BoxControllerDesc.halfHeight, expected 136 got %v", offset_of(BoxControllerDesc, halfHeight))
    testing.expectf(t, offset_of(BoxControllerDesc, halfSideExtent) == 140, "Wrong offset for BoxControllerDesc.halfSideExtent, expected 140 got %v", offset_of(BoxControllerDesc, halfSideExtent))
    testing.expectf(t, offset_of(BoxControllerDesc, halfForwardExtent) == 144, "Wrong offset for BoxControllerDesc.halfForwardExtent, expected 144 got %v", offset_of(BoxControllerDesc, halfForwardExtent))
    testing.expectf(t, size_of(BoxControllerDesc) == 152, "Wrong size for type BoxControllerDesc, expected 152 got %v", size_of(BoxControllerDesc))
}

@(test)
test_layout_BoxController :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BoxController) == 8, "Wrong size for type BoxController, expected 8 got %v", size_of(BoxController))
}

@(test)
test_layout_CapsuleControllerDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(CapsuleControllerDesc, radius) == 136, "Wrong offset for CapsuleControllerDesc.radius, expected 136 got %v", offset_of(CapsuleControllerDesc, radius))
    testing.expectf(t, offset_of(CapsuleControllerDesc, height) == 140, "Wrong offset for CapsuleControllerDesc.height, expected 140 got %v", offset_of(CapsuleControllerDesc, height))
    testing.expectf(t, offset_of(CapsuleControllerDesc, climbingMode) == 144, "Wrong offset for CapsuleControllerDesc.climbingMode, expected 144 got %v", offset_of(CapsuleControllerDesc, climbingMode))
    testing.expectf(t, size_of(CapsuleControllerDesc) == 152, "Wrong size for type CapsuleControllerDesc, expected 152 got %v", size_of(CapsuleControllerDesc))
}

@(test)
test_layout_CapsuleController :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CapsuleController) == 8, "Wrong size for type CapsuleController, expected 8 got %v", size_of(CapsuleController))
}

@(test)
test_layout_ControllerBehaviorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ControllerBehaviorCallback) == 8, "Wrong size for type ControllerBehaviorCallback, expected 8 got %v", size_of(ControllerBehaviorCallback))
}

@(test)
test_layout_ControllerManager :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ControllerManager) == 8, "Wrong size for type ControllerManager, expected 8 got %v", size_of(ControllerManager))
}

@(test)
test_layout_Dim3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Dim3, y) == 4, "Wrong offset for Dim3.y, expected 4 got %v", offset_of(Dim3, y))
    testing.expectf(t, offset_of(Dim3, z) == 8, "Wrong offset for Dim3.z, expected 8 got %v", offset_of(Dim3, z))
    testing.expectf(t, size_of(Dim3) == 12, "Wrong size for type Dim3, expected 12 got %v", size_of(Dim3))
}

@(test)
test_layout_SDFDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(SDFDesc, dims) == 24, "Wrong offset for SDFDesc.dims, expected 24 got %v", offset_of(SDFDesc, dims))
    testing.expectf(t, offset_of(SDFDesc, meshLower) == 36, "Wrong offset for SDFDesc.meshLower, expected 36 got %v", offset_of(SDFDesc, meshLower))
    testing.expectf(t, offset_of(SDFDesc, spacing) == 48, "Wrong offset for SDFDesc.spacing, expected 48 got %v", offset_of(SDFDesc, spacing))
    testing.expectf(t, offset_of(SDFDesc, subgridSize) == 52, "Wrong offset for SDFDesc.subgridSize, expected 52 got %v", offset_of(SDFDesc, subgridSize))
    testing.expectf(t, offset_of(SDFDesc, bitsPerSubgridPixel) == 56, "Wrong offset for SDFDesc.bitsPerSubgridPixel, expected 56 got %v", offset_of(SDFDesc, bitsPerSubgridPixel))
    testing.expectf(t, offset_of(SDFDesc, sdfSubgrids3DTexBlockDim) == 60, "Wrong offset for SDFDesc.sdfSubgrids3DTexBlockDim, expected 60 got %v", offset_of(SDFDesc, sdfSubgrids3DTexBlockDim))
    testing.expectf(t, offset_of(SDFDesc, sdfSubgrids) == 72, "Wrong offset for SDFDesc.sdfSubgrids, expected 72 got %v", offset_of(SDFDesc, sdfSubgrids))
    testing.expectf(t, offset_of(SDFDesc, sdfStartSlots) == 96, "Wrong offset for SDFDesc.sdfStartSlots, expected 96 got %v", offset_of(SDFDesc, sdfStartSlots))
    testing.expectf(t, offset_of(SDFDesc, subgridsMinSdfValue) == 120, "Wrong offset for SDFDesc.subgridsMinSdfValue, expected 120 got %v", offset_of(SDFDesc, subgridsMinSdfValue))
    testing.expectf(t, offset_of(SDFDesc, subgridsMaxSdfValue) == 124, "Wrong offset for SDFDesc.subgridsMaxSdfValue, expected 124 got %v", offset_of(SDFDesc, subgridsMaxSdfValue))
    testing.expectf(t, offset_of(SDFDesc, sdfBounds) == 128, "Wrong offset for SDFDesc.sdfBounds, expected 128 got %v", offset_of(SDFDesc, sdfBounds))
    testing.expectf(t, offset_of(SDFDesc, narrowBandThicknessRelativeToSdfBoundsDiagonal) == 152, "Wrong offset for SDFDesc.narrowBandThicknessRelativeToSdfBoundsDiagonal, expected 152 got %v", offset_of(SDFDesc, narrowBandThicknessRelativeToSdfBoundsDiagonal))
    testing.expectf(t, offset_of(SDFDesc, numThreadsForSdfConstruction) == 156, "Wrong offset for SDFDesc.numThreadsForSdfConstruction, expected 156 got %v", offset_of(SDFDesc, numThreadsForSdfConstruction))
    testing.expectf(t, size_of(SDFDesc) == 160, "Wrong size for type SDFDesc, expected 160 got %v", size_of(SDFDesc))
}

@(test)
test_layout_ConvexMeshDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(ConvexMeshDesc, polygons) == 24, "Wrong offset for ConvexMeshDesc.polygons, expected 24 got %v", offset_of(ConvexMeshDesc, polygons))
    testing.expectf(t, offset_of(ConvexMeshDesc, indices) == 48, "Wrong offset for ConvexMeshDesc.indices, expected 48 got %v", offset_of(ConvexMeshDesc, indices))
    testing.expectf(t, offset_of(ConvexMeshDesc, flags) == 72, "Wrong offset for ConvexMeshDesc.flags, expected 72 got %v", offset_of(ConvexMeshDesc, flags))
    testing.expectf(t, offset_of(ConvexMeshDesc, vertexLimit) == 74, "Wrong offset for ConvexMeshDesc.vertexLimit, expected 74 got %v", offset_of(ConvexMeshDesc, vertexLimit))
    testing.expectf(t, offset_of(ConvexMeshDesc, polygonLimit) == 76, "Wrong offset for ConvexMeshDesc.polygonLimit, expected 76 got %v", offset_of(ConvexMeshDesc, polygonLimit))
    testing.expectf(t, offset_of(ConvexMeshDesc, quantizedCount) == 78, "Wrong offset for ConvexMeshDesc.quantizedCount, expected 78 got %v", offset_of(ConvexMeshDesc, quantizedCount))
    testing.expectf(t, offset_of(ConvexMeshDesc, sdfDesc) == 80, "Wrong offset for ConvexMeshDesc.sdfDesc, expected 80 got %v", offset_of(ConvexMeshDesc, sdfDesc))
    testing.expectf(t, size_of(ConvexMeshDesc) == 88, "Wrong size for type ConvexMeshDesc, expected 88 got %v", size_of(ConvexMeshDesc))
}

@(test)
test_layout_TriangleMeshDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TriangleMeshDesc, sdfDesc) == 72, "Wrong offset for TriangleMeshDesc.sdfDesc, expected 72 got %v", offset_of(TriangleMeshDesc, sdfDesc))
    testing.expectf(t, size_of(TriangleMeshDesc) == 80, "Wrong size for type TriangleMeshDesc, expected 80 got %v", size_of(TriangleMeshDesc))
}

@(test)
test_layout_TetrahedronMeshDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(TetrahedronMeshDesc, points) == 16, "Wrong offset for TetrahedronMeshDesc.points, expected 16 got %v", offset_of(TetrahedronMeshDesc, points))
    testing.expectf(t, offset_of(TetrahedronMeshDesc, tetrahedrons) == 40, "Wrong offset for TetrahedronMeshDesc.tetrahedrons, expected 40 got %v", offset_of(TetrahedronMeshDesc, tetrahedrons))
    testing.expectf(t, offset_of(TetrahedronMeshDesc, flags) == 64, "Wrong offset for TetrahedronMeshDesc.flags, expected 64 got %v", offset_of(TetrahedronMeshDesc, flags))
    testing.expectf(t, offset_of(TetrahedronMeshDesc, tetsPerElement) == 66, "Wrong offset for TetrahedronMeshDesc.tetsPerElement, expected 66 got %v", offset_of(TetrahedronMeshDesc, tetsPerElement))
    testing.expectf(t, size_of(TetrahedronMeshDesc) == 72, "Wrong size for type TetrahedronMeshDesc, expected 72 got %v", size_of(TetrahedronMeshDesc))
}

@(test)
test_layout_SoftBodySimulationDataDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SoftBodySimulationDataDesc) == 24, "Wrong size for type SoftBodySimulationDataDesc, expected 24 got %v", size_of(SoftBodySimulationDataDesc))
}

@(test)
test_layout_BVH34MidphaseDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BVH34MidphaseDesc, buildStrategy) == 4, "Wrong offset for BVH34MidphaseDesc.buildStrategy, expected 4 got %v", offset_of(BVH34MidphaseDesc, buildStrategy))
    testing.expectf(t, offset_of(BVH34MidphaseDesc, quantized) == 8, "Wrong offset for BVH34MidphaseDesc.quantized, expected 8 got %v", offset_of(BVH34MidphaseDesc, quantized))
    testing.expectf(t, size_of(BVH34MidphaseDesc) == 12, "Wrong size for type BVH34MidphaseDesc, expected 12 got %v", size_of(BVH34MidphaseDesc))
}

@(test)
test_layout_MidphaseDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(MidphaseDesc) == 16, "Wrong size for type MidphaseDesc, expected 16 got %v", size_of(MidphaseDesc))
}

@(test)
test_layout_BVHDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(BVHDesc, enlargement) == 24, "Wrong offset for BVHDesc.enlargement, expected 24 got %v", offset_of(BVHDesc, enlargement))
    testing.expectf(t, offset_of(BVHDesc, numPrimsPerLeaf) == 28, "Wrong offset for BVHDesc.numPrimsPerLeaf, expected 28 got %v", offset_of(BVHDesc, numPrimsPerLeaf))
    testing.expectf(t, offset_of(BVHDesc, buildStrategy) == 32, "Wrong offset for BVHDesc.buildStrategy, expected 32 got %v", offset_of(BVHDesc, buildStrategy))
    testing.expectf(t, size_of(BVHDesc) == 40, "Wrong size for type BVHDesc, expected 40 got %v", size_of(BVHDesc))
}

@(test)
test_layout_CookingParams :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(CookingParams, planeTolerance) == 4, "Wrong offset for CookingParams.planeTolerance, expected 4 got %v", offset_of(CookingParams, planeTolerance))
    testing.expectf(t, offset_of(CookingParams, convexMeshCookingType) == 8, "Wrong offset for CookingParams.convexMeshCookingType, expected 8 got %v", offset_of(CookingParams, convexMeshCookingType))
    testing.expectf(t, offset_of(CookingParams, suppressTriangleMeshRemapTable) == 12, "Wrong offset for CookingParams.suppressTriangleMeshRemapTable, expected 12 got %v", offset_of(CookingParams, suppressTriangleMeshRemapTable))
    testing.expectf(t, offset_of(CookingParams, buildTriangleAdjacencies) == 13, "Wrong offset for CookingParams.buildTriangleAdjacencies, expected 13 got %v", offset_of(CookingParams, buildTriangleAdjacencies))
    testing.expectf(t, offset_of(CookingParams, buildGPUData) == 14, "Wrong offset for CookingParams.buildGPUData, expected 14 got %v", offset_of(CookingParams, buildGPUData))
    testing.expectf(t, offset_of(CookingParams, scale) == 16, "Wrong offset for CookingParams.scale, expected 16 got %v", offset_of(CookingParams, scale))
    testing.expectf(t, offset_of(CookingParams, meshPreprocessParams) == 24, "Wrong offset for CookingParams.meshPreprocessParams, expected 24 got %v", offset_of(CookingParams, meshPreprocessParams))
    testing.expectf(t, offset_of(CookingParams, meshWeldTolerance) == 28, "Wrong offset for CookingParams.meshWeldTolerance, expected 28 got %v", offset_of(CookingParams, meshWeldTolerance))
    testing.expectf(t, offset_of(CookingParams, midphaseDesc) == 32, "Wrong offset for CookingParams.midphaseDesc, expected 32 got %v", offset_of(CookingParams, midphaseDesc))
    testing.expectf(t, offset_of(CookingParams, gaussMapLimit) == 48, "Wrong offset for CookingParams.gaussMapLimit, expected 48 got %v", offset_of(CookingParams, gaussMapLimit))
    testing.expectf(t, offset_of(CookingParams, maxWeightRatioInTet) == 52, "Wrong offset for CookingParams.maxWeightRatioInTet, expected 52 got %v", offset_of(CookingParams, maxWeightRatioInTet))
    testing.expectf(t, size_of(CookingParams) == 56, "Wrong size for type CookingParams, expected 56 got %v", size_of(CookingParams))
}

@(test)
test_layout_DefaultMemoryOutputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultMemoryOutputStream) == 32, "Wrong size for type DefaultMemoryOutputStream, expected 32 got %v", size_of(DefaultMemoryOutputStream))
}

@(test)
test_layout_DefaultMemoryInputData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultMemoryInputData) == 32, "Wrong size for type DefaultMemoryInputData, expected 32 got %v", size_of(DefaultMemoryInputData))
}

@(test)
test_layout_DefaultFileOutputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultFileOutputStream) == 16, "Wrong size for type DefaultFileOutputStream, expected 16 got %v", size_of(DefaultFileOutputStream))
}

@(test)
test_layout_DefaultFileInputData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultFileInputData) == 24, "Wrong size for type DefaultFileInputData, expected 24 got %v", size_of(DefaultFileInputData))
}

@(test)
test_layout_DefaultAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultAllocator) == 8, "Wrong size for type DefaultAllocator, expected 8 got %v", size_of(DefaultAllocator))
}

@(test)
test_layout_Joint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Joint, userData) == 16, "Wrong offset for Joint.userData, expected 16 got %v", offset_of(Joint, userData))
    testing.expectf(t, size_of(Joint) == 24, "Wrong size for type Joint, expected 24 got %v", size_of(Joint))
}

@(test)
test_layout_Spring :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Spring, damping) == 4, "Wrong offset for Spring.damping, expected 4 got %v", offset_of(Spring, damping))
    testing.expectf(t, size_of(Spring) == 8, "Wrong size for type Spring, expected 8 got %v", size_of(Spring))
}

@(test)
test_layout_DistanceJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DistanceJoint) == 24, "Wrong size for type DistanceJoint, expected 24 got %v", size_of(DistanceJoint))
}

@(test)
test_layout_JacobianRow :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JacobianRow, linear1) == 12, "Wrong offset for JacobianRow.linear1, expected 12 got %v", offset_of(JacobianRow, linear1))
    testing.expectf(t, offset_of(JacobianRow, angular0) == 24, "Wrong offset for JacobianRow.angular0, expected 24 got %v", offset_of(JacobianRow, angular0))
    testing.expectf(t, offset_of(JacobianRow, angular1) == 36, "Wrong offset for JacobianRow.angular1, expected 36 got %v", offset_of(JacobianRow, angular1))
    testing.expectf(t, size_of(JacobianRow) == 48, "Wrong size for type JacobianRow, expected 48 got %v", size_of(JacobianRow))
}

@(test)
test_layout_ContactJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(ContactJoint) == 24, "Wrong size for type ContactJoint, expected 24 got %v", size_of(ContactJoint))
}

@(test)
test_layout_FixedJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(FixedJoint) == 24, "Wrong size for type FixedJoint, expected 24 got %v", size_of(FixedJoint))
}

@(test)
test_layout_JointLimitParameters :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JointLimitParameters, bounceThreshold) == 4, "Wrong offset for JointLimitParameters.bounceThreshold, expected 4 got %v", offset_of(JointLimitParameters, bounceThreshold))
    testing.expectf(t, offset_of(JointLimitParameters, stiffness) == 8, "Wrong offset for JointLimitParameters.stiffness, expected 8 got %v", offset_of(JointLimitParameters, stiffness))
    testing.expectf(t, offset_of(JointLimitParameters, damping) == 12, "Wrong offset for JointLimitParameters.damping, expected 12 got %v", offset_of(JointLimitParameters, damping))
    testing.expectf(t, offset_of(JointLimitParameters, contactDistance_deprecated) == 16, "Wrong offset for JointLimitParameters.contactDistance_deprecated, expected 16 got %v", offset_of(JointLimitParameters, contactDistance_deprecated))
    testing.expectf(t, size_of(JointLimitParameters) == 20, "Wrong size for type JointLimitParameters, expected 20 got %v", size_of(JointLimitParameters))
}

@(test)
test_layout_JointLinearLimit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JointLinearLimit, value) == 20, "Wrong offset for JointLinearLimit.value, expected 20 got %v", offset_of(JointLinearLimit, value))
    testing.expectf(t, size_of(JointLinearLimit) == 24, "Wrong size for type JointLinearLimit, expected 24 got %v", size_of(JointLinearLimit))
}

@(test)
test_layout_JointLinearLimitPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JointLinearLimitPair, upper) == 20, "Wrong offset for JointLinearLimitPair.upper, expected 20 got %v", offset_of(JointLinearLimitPair, upper))
    testing.expectf(t, offset_of(JointLinearLimitPair, lower) == 24, "Wrong offset for JointLinearLimitPair.lower, expected 24 got %v", offset_of(JointLinearLimitPair, lower))
    testing.expectf(t, size_of(JointLinearLimitPair) == 28, "Wrong size for type JointLinearLimitPair, expected 28 got %v", size_of(JointLinearLimitPair))
}

@(test)
test_layout_JointAngularLimitPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JointAngularLimitPair, upper) == 20, "Wrong offset for JointAngularLimitPair.upper, expected 20 got %v", offset_of(JointAngularLimitPair, upper))
    testing.expectf(t, offset_of(JointAngularLimitPair, lower) == 24, "Wrong offset for JointAngularLimitPair.lower, expected 24 got %v", offset_of(JointAngularLimitPair, lower))
    testing.expectf(t, size_of(JointAngularLimitPair) == 28, "Wrong size for type JointAngularLimitPair, expected 28 got %v", size_of(JointAngularLimitPair))
}

@(test)
test_layout_JointLimitCone :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JointLimitCone, yAngle) == 20, "Wrong offset for JointLimitCone.yAngle, expected 20 got %v", offset_of(JointLimitCone, yAngle))
    testing.expectf(t, offset_of(JointLimitCone, zAngle) == 24, "Wrong offset for JointLimitCone.zAngle, expected 24 got %v", offset_of(JointLimitCone, zAngle))
    testing.expectf(t, size_of(JointLimitCone) == 28, "Wrong size for type JointLimitCone, expected 28 got %v", size_of(JointLimitCone))
}

@(test)
test_layout_JointLimitPyramid :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(JointLimitPyramid, yAngleMin) == 20, "Wrong offset for JointLimitPyramid.yAngleMin, expected 20 got %v", offset_of(JointLimitPyramid, yAngleMin))
    testing.expectf(t, offset_of(JointLimitPyramid, yAngleMax) == 24, "Wrong offset for JointLimitPyramid.yAngleMax, expected 24 got %v", offset_of(JointLimitPyramid, yAngleMax))
    testing.expectf(t, offset_of(JointLimitPyramid, zAngleMin) == 28, "Wrong offset for JointLimitPyramid.zAngleMin, expected 28 got %v", offset_of(JointLimitPyramid, zAngleMin))
    testing.expectf(t, offset_of(JointLimitPyramid, zAngleMax) == 32, "Wrong offset for JointLimitPyramid.zAngleMax, expected 32 got %v", offset_of(JointLimitPyramid, zAngleMax))
    testing.expectf(t, size_of(JointLimitPyramid) == 36, "Wrong size for type JointLimitPyramid, expected 36 got %v", size_of(JointLimitPyramid))
}

@(test)
test_layout_PrismaticJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PrismaticJoint) == 24, "Wrong size for type PrismaticJoint, expected 24 got %v", size_of(PrismaticJoint))
}

@(test)
test_layout_RevoluteJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RevoluteJoint) == 24, "Wrong size for type RevoluteJoint, expected 24 got %v", size_of(RevoluteJoint))
}

@(test)
test_layout_SphericalJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(SphericalJoint) == 24, "Wrong size for type SphericalJoint, expected 24 got %v", size_of(SphericalJoint))
}

@(test)
test_layout_D6JointDrive :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(D6JointDrive, forceLimit) == 8, "Wrong offset for D6JointDrive.forceLimit, expected 8 got %v", offset_of(D6JointDrive, forceLimit))
    testing.expectf(t, offset_of(D6JointDrive, flags) == 12, "Wrong offset for D6JointDrive.flags, expected 12 got %v", offset_of(D6JointDrive, flags))
    testing.expectf(t, size_of(D6JointDrive) == 16, "Wrong size for type D6JointDrive, expected 16 got %v", size_of(D6JointDrive))
}

@(test)
test_layout_D6Joint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(D6Joint) == 24, "Wrong size for type D6Joint, expected 24 got %v", size_of(D6Joint))
}

@(test)
test_layout_GearJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(GearJoint) == 24, "Wrong size for type GearJoint, expected 24 got %v", size_of(GearJoint))
}

@(test)
test_layout_RackAndPinionJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RackAndPinionJoint) == 24, "Wrong size for type RackAndPinionJoint, expected 24 got %v", size_of(RackAndPinionJoint))
}

@(test)
test_layout_GroupsMask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(GroupsMask, bits1) == 2, "Wrong offset for GroupsMask.bits1, expected 2 got %v", offset_of(GroupsMask, bits1))
    testing.expectf(t, offset_of(GroupsMask, bits2) == 4, "Wrong offset for GroupsMask.bits2, expected 4 got %v", offset_of(GroupsMask, bits2))
    testing.expectf(t, offset_of(GroupsMask, bits3) == 6, "Wrong offset for GroupsMask.bits3, expected 6 got %v", offset_of(GroupsMask, bits3))
    testing.expectf(t, size_of(GroupsMask) == 8, "Wrong size for type GroupsMask, expected 8 got %v", size_of(GroupsMask))
}

@(test)
test_layout_DefaultErrorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultErrorCallback) == 8, "Wrong size for type DefaultErrorCallback, expected 8 got %v", size_of(DefaultErrorCallback))
}


@(test)
test_layout_MassProperties :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(MassProperties, centerOfMass) == 36, "Wrong offset for MassProperties.centerOfMass, expected 36 got %v", offset_of(MassProperties, centerOfMass))
    testing.expectf(t, offset_of(MassProperties, mass) == 48, "Wrong offset for MassProperties.mass, expected 48 got %v", offset_of(MassProperties, mass))
    testing.expectf(t, size_of(MassProperties) == 52, "Wrong size for type MassProperties, expected 52 got %v", size_of(MassProperties))
}



@(test)
test_layout_MeshOverlapUtil :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(MeshOverlapUtil) == 1040, "Wrong size for type MeshOverlapUtil, expected 1040 got %v", size_of(MeshOverlapUtil))
}

@(test)
test_layout_XmlMiscParameter :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(XmlMiscParameter, scale) == 12, "Wrong offset for XmlMiscParameter.scale, expected 12 got %v", offset_of(XmlMiscParameter, scale))
    testing.expectf(t, size_of(XmlMiscParameter) == 20, "Wrong size for type XmlMiscParameter, expected 20 got %v", size_of(XmlMiscParameter))
}


@(test)
test_layout_DefaultCpuDispatcher :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(DefaultCpuDispatcher) == 8, "Wrong size for type DefaultCpuDispatcher, expected 8 got %v", size_of(DefaultCpuDispatcher))
}




@(test)
test_layout_BatchQueryExt :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(BatchQueryExt) == 8, "Wrong size for type BatchQueryExt, expected 8 got %v", size_of(BatchQueryExt))
}

@(test)
test_layout_CustomSceneQuerySystem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CustomSceneQuerySystem) == 8, "Wrong size for type CustomSceneQuerySystem, expected 8 got %v", size_of(CustomSceneQuerySystem))
}

@(test)
test_layout_CustomSceneQuerySystemAdapter :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(CustomSceneQuerySystemAdapter) == 8, "Wrong size for type CustomSceneQuerySystemAdapter, expected 8 got %v", size_of(CustomSceneQuerySystemAdapter))
}


@(test)
test_layout_PoissonSampler :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PoissonSampler) == 8, "Wrong size for type PoissonSampler, expected 8 got %v", size_of(PoissonSampler))
}

@(test)
test_layout_TriangleMeshPoissonSampler :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(TriangleMeshPoissonSampler) == 24, "Wrong size for type TriangleMeshPoissonSampler, expected 24 got %v", size_of(TriangleMeshPoissonSampler))
}


@(test)
test_layout_RepXObject :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(RepXObject, serializable) == 8, "Wrong offset for RepXObject.serializable, expected 8 got %v", offset_of(RepXObject, serializable))
    testing.expectf(t, offset_of(RepXObject, id) == 16, "Wrong offset for RepXObject.id, expected 16 got %v", offset_of(RepXObject, id))
    testing.expectf(t, size_of(RepXObject) == 24, "Wrong size for type RepXObject, expected 24 got %v", size_of(RepXObject))
}

@(test)
test_layout_RepXInstantiationArgs :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(RepXInstantiationArgs, cooker) == 8, "Wrong offset for RepXInstantiationArgs.cooker, expected 8 got %v", offset_of(RepXInstantiationArgs, cooker))
    testing.expectf(t, offset_of(RepXInstantiationArgs, stringTable) == 16, "Wrong offset for RepXInstantiationArgs.stringTable, expected 16 got %v", offset_of(RepXInstantiationArgs, stringTable))
    testing.expectf(t, size_of(RepXInstantiationArgs) == 24, "Wrong size for type RepXInstantiationArgs, expected 24 got %v", size_of(RepXInstantiationArgs))
}

@(test)
test_layout_RepXSerializer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(RepXSerializer) == 8, "Wrong size for type RepXSerializer, expected 8 got %v", size_of(RepXSerializer))
}

@(test)
test_layout_Pvd :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(Pvd) == 8, "Wrong size for type Pvd, expected 8 got %v", size_of(Pvd))
}

@(test)
test_layout_PvdTransport :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PvdTransport) == 8, "Wrong size for type PvdTransport, expected 8 got %v", size_of(PvdTransport))
}
