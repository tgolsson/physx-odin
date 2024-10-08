package tests
import  "core:testing"
import physx ".."

@(test)
test_layout_PxAllocatorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxAllocatorCallback) == 8, "Wrong size for type PxAllocatorCallback, expected 8 got %v", size_of(PxAllocatorCallback))
}

@(test)
test_layout_PxAssertHandler :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxAssertHandler) == 8, "Wrong size for type PxAssertHandler, expected 8 got %v", size_of(PxAssertHandler))
}

@(test)
test_layout_PxFoundation :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxFoundation) == 8, "Wrong size for type PxFoundation, expected 8 got %v", size_of(PxFoundation))
}



@(test)
test_layout_PxVirtualAllocatorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxVirtualAllocatorCallback) == 8, "Wrong size for type PxVirtualAllocatorCallback, expected 8 got %v", size_of(PxVirtualAllocatorCallback))
}

@(test)
test_layout_PxVirtualAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxVirtualAllocator) == 16, "Wrong size for type PxVirtualAllocator, expected 16 got %v", size_of(PxVirtualAllocator))
}




@(test)
test_layout_PxBitAndByte :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBitAndByte) == 1, "Wrong size for type PxBitAndByte, expected 1 got %v", size_of(PxBitAndByte))
}

@(test)
test_layout_PxBitMap :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBitMap) == 16, "Wrong size for type PxBitMap, expected 16 got %v", size_of(PxBitMap))
}

@(test)
test_layout_PxVec3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxVec3, y) == 4, "Wrong offset for PxVec3.y, expected 4 got %v", offset_of(PxVec3, y))
    testing.expectf(t, offset_of(PxVec3, z) == 8, "Wrong offset for PxVec3.z, expected 8 got %v", offset_of(PxVec3, z))
    testing.expectf(t, size_of(PxVec3) == 12, "Wrong size for type PxVec3, expected 12 got %v", size_of(PxVec3))
}

@(test)
test_layout_PxVec3Padded :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxVec3Padded, padding) == 12, "Wrong offset for PxVec3Padded.padding, expected 12 got %v", offset_of(PxVec3Padded, padding))
    testing.expectf(t, size_of(PxVec3Padded) == 16, "Wrong size for type PxVec3Padded, expected 16 got %v", size_of(PxVec3Padded))
}

@(test)
test_layout_PxQuat :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxQuat, y) == 4, "Wrong offset for PxQuat.y, expected 4 got %v", offset_of(PxQuat, y))
    testing.expectf(t, offset_of(PxQuat, z) == 8, "Wrong offset for PxQuat.z, expected 8 got %v", offset_of(PxQuat, z))
    testing.expectf(t, offset_of(PxQuat, w) == 12, "Wrong offset for PxQuat.w, expected 12 got %v", offset_of(PxQuat, w))
    testing.expectf(t, size_of(PxQuat) == 16, "Wrong size for type PxQuat, expected 16 got %v", size_of(PxQuat))
}

@(test)
test_layout_PxTransform :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTransform, p) == 16, "Wrong offset for PxTransform.p, expected 16 got %v", offset_of(PxTransform, p))
    testing.expectf(t, size_of(PxTransform) == 28, "Wrong size for type PxTransform, expected 28 got %v", size_of(PxTransform))
}

@(test)
test_layout_PxTransformPadded :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTransformPadded, padding) == 28, "Wrong offset for PxTransformPadded.padding, expected 28 got %v", offset_of(PxTransformPadded, padding))
    testing.expectf(t, size_of(PxTransformPadded) == 32, "Wrong size for type PxTransformPadded, expected 32 got %v", size_of(PxTransformPadded))
}

@(test)
test_layout_PxMat33 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxMat33, column1) == 12, "Wrong offset for PxMat33.column1, expected 12 got %v", offset_of(PxMat33, column1))
    testing.expectf(t, offset_of(PxMat33, column2) == 24, "Wrong offset for PxMat33.column2, expected 24 got %v", offset_of(PxMat33, column2))
    testing.expectf(t, size_of(PxMat33) == 36, "Wrong size for type PxMat33, expected 36 got %v", size_of(PxMat33))
}

@(test)
test_layout_PxBounds3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBounds3, maximum) == 12, "Wrong offset for PxBounds3.maximum, expected 12 got %v", offset_of(PxBounds3, maximum))
    testing.expectf(t, size_of(PxBounds3) == 24, "Wrong size for type PxBounds3, expected 24 got %v", size_of(PxBounds3))
}

@(test)
test_layout_PxErrorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxErrorCallback) == 8, "Wrong size for type PxErrorCallback, expected 8 got %v", size_of(PxErrorCallback))
}

@(test)
test_layout_PxAllocationListener :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxAllocationListener) == 8, "Wrong size for type PxAllocationListener, expected 8 got %v", size_of(PxAllocationListener))
}

@(test)
test_layout_PxBroadcastingAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBroadcastingAllocator) == 176, "Wrong size for type PxBroadcastingAllocator, expected 176 got %v", size_of(PxBroadcastingAllocator))
}

@(test)
test_layout_PxBroadcastingErrorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBroadcastingErrorCallback) == 160, "Wrong size for type PxBroadcastingErrorCallback, expected 160 got %v", size_of(PxBroadcastingErrorCallback))
}

@(test)
test_layout_PxInputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxInputStream) == 8, "Wrong size for type PxInputStream, expected 8 got %v", size_of(PxInputStream))
}

@(test)
test_layout_PxInputData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxInputData) == 8, "Wrong size for type PxInputData, expected 8 got %v", size_of(PxInputData))
}

@(test)
test_layout_PxOutputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxOutputStream) == 8, "Wrong size for type PxOutputStream, expected 8 got %v", size_of(PxOutputStream))
}

@(test)
test_layout_PxVec4 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxVec4, y) == 4, "Wrong offset for PxVec4.y, expected 4 got %v", offset_of(PxVec4, y))
    testing.expectf(t, offset_of(PxVec4, z) == 8, "Wrong offset for PxVec4.z, expected 8 got %v", offset_of(PxVec4, z))
    testing.expectf(t, offset_of(PxVec4, w) == 12, "Wrong offset for PxVec4.w, expected 12 got %v", offset_of(PxVec4, w))
    testing.expectf(t, size_of(PxVec4) == 16, "Wrong size for type PxVec4, expected 16 got %v", size_of(PxVec4))
}

@(test)
test_layout_PxMat44 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxMat44, column1) == 16, "Wrong offset for PxMat44.column1, expected 16 got %v", offset_of(PxMat44, column1))
    testing.expectf(t, offset_of(PxMat44, column2) == 32, "Wrong offset for PxMat44.column2, expected 32 got %v", offset_of(PxMat44, column2))
    testing.expectf(t, offset_of(PxMat44, column3) == 48, "Wrong offset for PxMat44.column3, expected 48 got %v", offset_of(PxMat44, column3))
    testing.expectf(t, size_of(PxMat44) == 64, "Wrong size for type PxMat44, expected 64 got %v", size_of(PxMat44))
}

@(test)
test_layout_PxPlane :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxPlane, d) == 12, "Wrong offset for PxPlane.d, expected 12 got %v", offset_of(PxPlane, d))
    testing.expectf(t, size_of(PxPlane) == 16, "Wrong size for type PxPlane, expected 16 got %v", size_of(PxPlane))
}



@(test)
test_layout_PxReadWriteLock :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxReadWriteLock) == 8, "Wrong size for type PxReadWriteLock, expected 8 got %v", size_of(PxReadWriteLock))
}

@(test)
test_layout_PxProfilerCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxProfilerCallback) == 8, "Wrong size for type PxProfilerCallback, expected 8 got %v", size_of(PxProfilerCallback))
}

@(test)
test_layout_PxProfileScoped :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxProfileScoped, mEventName) == 8, "Wrong offset for PxProfileScoped.mEventName, expected 8 got %v", offset_of(PxProfileScoped, mEventName))
    testing.expectf(t, offset_of(PxProfileScoped, mProfilerData) == 16, "Wrong offset for PxProfileScoped.mProfilerData, expected 16 got %v", offset_of(PxProfileScoped, mProfilerData))
    testing.expectf(t, offset_of(PxProfileScoped, mContextId) == 24, "Wrong offset for PxProfileScoped.mContextId, expected 24 got %v", offset_of(PxProfileScoped, mContextId))
    testing.expectf(t, offset_of(PxProfileScoped, mDetached) == 32, "Wrong offset for PxProfileScoped.mDetached, expected 32 got %v", offset_of(PxProfileScoped, mDetached))
    testing.expectf(t, size_of(PxProfileScoped) == 40, "Wrong size for type PxProfileScoped, expected 40 got %v", size_of(PxProfileScoped))
}

@(test)
test_layout_PxSListEntry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSListEntry) == 16, "Wrong size for type PxSListEntry, expected 16 got %v", size_of(PxSListEntry))
}



@(test)
test_layout_PxRunnable :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRunnable) == 8, "Wrong size for type PxRunnable, expected 8 got %v", size_of(PxRunnable))
}

@(test)
test_layout_PxCounterFrequencyToTensOfNanos :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxCounterFrequencyToTensOfNanos, mDenominator) == 8, "Wrong offset for PxCounterFrequencyToTensOfNanos.mDenominator, expected 8 got %v", offset_of(PxCounterFrequencyToTensOfNanos, mDenominator))
    testing.expectf(t, size_of(PxCounterFrequencyToTensOfNanos) == 16, "Wrong size for type PxCounterFrequencyToTensOfNanos, expected 16 got %v", size_of(PxCounterFrequencyToTensOfNanos))
}

@(test)
test_layout_PxTime :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTime) == 8, "Wrong size for type PxTime, expected 8 got %v", size_of(PxTime))
}

@(test)
test_layout_PxVec2 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxVec2, y) == 4, "Wrong offset for PxVec2.y, expected 4 got %v", offset_of(PxVec2, y))
    testing.expectf(t, size_of(PxVec2) == 8, "Wrong size for type PxVec2, expected 8 got %v", size_of(PxVec2))
}

@(test)
test_layout_PxStridedData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxStridedData, data) == 8, "Wrong offset for PxStridedData.data, expected 8 got %v", offset_of(PxStridedData, data))
    testing.expectf(t, size_of(PxStridedData) == 16, "Wrong size for type PxStridedData, expected 16 got %v", size_of(PxStridedData))
}

@(test)
test_layout_PxBoundedData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBoundedData, count) == 16, "Wrong offset for PxBoundedData.count, expected 16 got %v", offset_of(PxBoundedData, count))
    testing.expectf(t, size_of(PxBoundedData) == 24, "Wrong size for type PxBoundedData, expected 24 got %v", size_of(PxBoundedData))
}

@(test)
test_layout_PxDebugPoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDebugPoint, color) == 12, "Wrong offset for PxDebugPoint.color, expected 12 got %v", offset_of(PxDebugPoint, color))
    testing.expectf(t, size_of(PxDebugPoint) == 16, "Wrong size for type PxDebugPoint, expected 16 got %v", size_of(PxDebugPoint))
}

@(test)
test_layout_PxDebugLine :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDebugLine, color0) == 12, "Wrong offset for PxDebugLine.color0, expected 12 got %v", offset_of(PxDebugLine, color0))
    testing.expectf(t, offset_of(PxDebugLine, pos1) == 16, "Wrong offset for PxDebugLine.pos1, expected 16 got %v", offset_of(PxDebugLine, pos1))
    testing.expectf(t, offset_of(PxDebugLine, color1) == 28, "Wrong offset for PxDebugLine.color1, expected 28 got %v", offset_of(PxDebugLine, color1))
    testing.expectf(t, size_of(PxDebugLine) == 32, "Wrong size for type PxDebugLine, expected 32 got %v", size_of(PxDebugLine))
}

@(test)
test_layout_PxDebugTriangle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDebugTriangle, color0) == 12, "Wrong offset for PxDebugTriangle.color0, expected 12 got %v", offset_of(PxDebugTriangle, color0))
    testing.expectf(t, offset_of(PxDebugTriangle, pos1) == 16, "Wrong offset for PxDebugTriangle.pos1, expected 16 got %v", offset_of(PxDebugTriangle, pos1))
    testing.expectf(t, offset_of(PxDebugTriangle, color1) == 28, "Wrong offset for PxDebugTriangle.color1, expected 28 got %v", offset_of(PxDebugTriangle, color1))
    testing.expectf(t, offset_of(PxDebugTriangle, pos2) == 32, "Wrong offset for PxDebugTriangle.pos2, expected 32 got %v", offset_of(PxDebugTriangle, pos2))
    testing.expectf(t, offset_of(PxDebugTriangle, color2) == 44, "Wrong offset for PxDebugTriangle.color2, expected 44 got %v", offset_of(PxDebugTriangle, color2))
    testing.expectf(t, size_of(PxDebugTriangle) == 48, "Wrong size for type PxDebugTriangle, expected 48 got %v", size_of(PxDebugTriangle))
}

@(test)
test_layout_PxDebugText :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDebugText, size) == 12, "Wrong offset for PxDebugText.size, expected 12 got %v", offset_of(PxDebugText, size))
    testing.expectf(t, offset_of(PxDebugText, color) == 16, "Wrong offset for PxDebugText.color, expected 16 got %v", offset_of(PxDebugText, color))
    testing.expectf(t, offset_of(PxDebugText, string) == 24, "Wrong offset for PxDebugText.string, expected 24 got %v", offset_of(PxDebugText, string))
    testing.expectf(t, size_of(PxDebugText) == 32, "Wrong size for type PxDebugText, expected 32 got %v", size_of(PxDebugText))
}

@(test)
test_layout_PxRenderBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRenderBuffer) == 8, "Wrong size for type PxRenderBuffer, expected 8 got %v", size_of(PxRenderBuffer))
}

@(test)
test_layout_PxProcessPxBaseCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxProcessPxBaseCallback) == 8, "Wrong size for type PxProcessPxBaseCallback, expected 8 got %v", size_of(PxProcessPxBaseCallback))
}

@(test)
test_layout_PxSerializationContext :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSerializationContext) == 8, "Wrong size for type PxSerializationContext, expected 8 got %v", size_of(PxSerializationContext))
}

@(test)
test_layout_PxDeserializationContext :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDeserializationContext) == 16, "Wrong size for type PxDeserializationContext, expected 16 got %v", size_of(PxDeserializationContext))
}

@(test)
test_layout_PxSerializationRegistry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSerializationRegistry) == 8, "Wrong size for type PxSerializationRegistry, expected 8 got %v", size_of(PxSerializationRegistry))
}

@(test)
test_layout_PxCollection :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCollection) == 8, "Wrong size for type PxCollection, expected 8 got %v", size_of(PxCollection))
}

@(test)
test_layout_PxBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBase) == 16, "Wrong size for type PxBase, expected 16 got %v", size_of(PxBase))
}

@(test)
test_layout_PxRefCounted :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRefCounted) == 16, "Wrong size for type PxRefCounted, expected 16 got %v", size_of(PxRefCounted))
}

@(test)
test_layout_PxTolerancesScale :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTolerancesScale, speed) == 4, "Wrong offset for PxTolerancesScale.speed, expected 4 got %v", offset_of(PxTolerancesScale, speed))
    testing.expectf(t, size_of(PxTolerancesScale) == 8, "Wrong size for type PxTolerancesScale, expected 8 got %v", size_of(PxTolerancesScale))
}

@(test)
test_layout_PxStringTable :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxStringTable) == 8, "Wrong size for type PxStringTable, expected 8 got %v", size_of(PxStringTable))
}

@(test)
test_layout_PxSerializer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSerializer) == 8, "Wrong size for type PxSerializer, expected 8 got %v", size_of(PxSerializer))
}

@(test)
test_layout_PxMetaDataEntry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxMetaDataEntry, name) == 8, "Wrong offset for PxMetaDataEntry.name, expected 8 got %v", offset_of(PxMetaDataEntry, name))
    testing.expectf(t, offset_of(PxMetaDataEntry, offset) == 16, "Wrong offset for PxMetaDataEntry.offset, expected 16 got %v", offset_of(PxMetaDataEntry, offset))
    testing.expectf(t, offset_of(PxMetaDataEntry, size) == 20, "Wrong offset for PxMetaDataEntry.size, expected 20 got %v", offset_of(PxMetaDataEntry, size))
    testing.expectf(t, offset_of(PxMetaDataEntry, count) == 24, "Wrong offset for PxMetaDataEntry.count, expected 24 got %v", offset_of(PxMetaDataEntry, count))
    testing.expectf(t, offset_of(PxMetaDataEntry, offsetSize) == 28, "Wrong offset for PxMetaDataEntry.offsetSize, expected 28 got %v", offset_of(PxMetaDataEntry, offsetSize))
    testing.expectf(t, offset_of(PxMetaDataEntry, flags) == 32, "Wrong offset for PxMetaDataEntry.flags, expected 32 got %v", offset_of(PxMetaDataEntry, flags))
    testing.expectf(t, offset_of(PxMetaDataEntry, alignment) == 36, "Wrong offset for PxMetaDataEntry.alignment, expected 36 got %v", offset_of(PxMetaDataEntry, alignment))
    testing.expectf(t, size_of(PxMetaDataEntry) == 40, "Wrong size for type PxMetaDataEntry, expected 40 got %v", size_of(PxMetaDataEntry))
}

@(test)
test_layout_PxInsertionCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxInsertionCallback) == 8, "Wrong size for type PxInsertionCallback, expected 8 got %v", size_of(PxInsertionCallback))
}

@(test)
test_layout_PxTaskManager :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTaskManager) == 8, "Wrong size for type PxTaskManager, expected 8 got %v", size_of(PxTaskManager))
}

@(test)
test_layout_PxCpuDispatcher :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCpuDispatcher) == 8, "Wrong size for type PxCpuDispatcher, expected 8 got %v", size_of(PxCpuDispatcher))
}

@(test)
test_layout_PxBaseTask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBaseTask) == 24, "Wrong size for type PxBaseTask, expected 24 got %v", size_of(PxBaseTask))
}

@(test)
test_layout_PxTask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTask) == 32, "Wrong size for type PxTask, expected 32 got %v", size_of(PxTask))
}

@(test)
test_layout_PxLightCpuTask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxLightCpuTask) == 40, "Wrong size for type PxLightCpuTask, expected 40 got %v", size_of(PxLightCpuTask))
}

@(test)
test_layout_PxGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGeometry, mTypePadding) == 4, "Wrong offset for PxGeometry.mTypePadding, expected 4 got %v", offset_of(PxGeometry, mTypePadding))
    testing.expectf(t, size_of(PxGeometry) == 8, "Wrong size for type PxGeometry, expected 8 got %v", size_of(PxGeometry))
}

@(test)
test_layout_PxBoxGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBoxGeometry, halfExtents) == 8, "Wrong offset for PxBoxGeometry.halfExtents, expected 8 got %v", offset_of(PxBoxGeometry, halfExtents))
    testing.expectf(t, size_of(PxBoxGeometry) == 20, "Wrong size for type PxBoxGeometry, expected 20 got %v", size_of(PxBoxGeometry))
}

@(test)
test_layout_PxBVHRaycastCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBVHRaycastCallback) == 8, "Wrong size for type PxBVHRaycastCallback, expected 8 got %v", size_of(PxBVHRaycastCallback))
}

@(test)
test_layout_PxBVHOverlapCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBVHOverlapCallback) == 8, "Wrong size for type PxBVHOverlapCallback, expected 8 got %v", size_of(PxBVHOverlapCallback))
}

@(test)
test_layout_PxBVHTraversalCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBVHTraversalCallback) == 8, "Wrong size for type PxBVHTraversalCallback, expected 8 got %v", size_of(PxBVHTraversalCallback))
}

@(test)
test_layout_PxBVH :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBVH) == 16, "Wrong size for type PxBVH, expected 16 got %v", size_of(PxBVH))
}

@(test)
test_layout_PxCapsuleGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxCapsuleGeometry, radius) == 8, "Wrong offset for PxCapsuleGeometry.radius, expected 8 got %v", offset_of(PxCapsuleGeometry, radius))
    testing.expectf(t, offset_of(PxCapsuleGeometry, halfHeight) == 12, "Wrong offset for PxCapsuleGeometry.halfHeight, expected 12 got %v", offset_of(PxCapsuleGeometry, halfHeight))
    testing.expectf(t, size_of(PxCapsuleGeometry) == 16, "Wrong size for type PxCapsuleGeometry, expected 16 got %v", size_of(PxCapsuleGeometry))
}

@(test)
test_layout_PxHullPolygon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxHullPolygon, mNbVerts) == 16, "Wrong offset for PxHullPolygon.mNbVerts, expected 16 got %v", offset_of(PxHullPolygon, mNbVerts))
    testing.expectf(t, offset_of(PxHullPolygon, mIndexBase) == 18, "Wrong offset for PxHullPolygon.mIndexBase, expected 18 got %v", offset_of(PxHullPolygon, mIndexBase))
    testing.expectf(t, size_of(PxHullPolygon) == 20, "Wrong size for type PxHullPolygon, expected 20 got %v", size_of(PxHullPolygon))
}

@(test)
test_layout_PxConvexMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxConvexMesh) == 16, "Wrong size for type PxConvexMesh, expected 16 got %v", size_of(PxConvexMesh))
}

@(test)
test_layout_PxMeshScale :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxMeshScale, rotation) == 12, "Wrong offset for PxMeshScale.rotation, expected 12 got %v", offset_of(PxMeshScale, rotation))
    testing.expectf(t, size_of(PxMeshScale) == 28, "Wrong size for type PxMeshScale, expected 28 got %v", size_of(PxMeshScale))
}

@(test)
test_layout_PxConvexMeshGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConvexMeshGeometry, scale) == 8, "Wrong offset for PxConvexMeshGeometry.scale, expected 8 got %v", offset_of(PxConvexMeshGeometry, scale))
    testing.expectf(t, offset_of(PxConvexMeshGeometry, convexMesh) == 40, "Wrong offset for PxConvexMeshGeometry.convexMesh, expected 40 got %v", offset_of(PxConvexMeshGeometry, convexMesh))
    testing.expectf(t, offset_of(PxConvexMeshGeometry, meshFlags) == 48, "Wrong offset for PxConvexMeshGeometry.meshFlags, expected 48 got %v", offset_of(PxConvexMeshGeometry, meshFlags))
    testing.expectf(t, size_of(PxConvexMeshGeometry) == 56, "Wrong size for type PxConvexMeshGeometry, expected 56 got %v", size_of(PxConvexMeshGeometry))
}

@(test)
test_layout_PxSphereGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSphereGeometry, radius) == 8, "Wrong offset for PxSphereGeometry.radius, expected 8 got %v", offset_of(PxSphereGeometry, radius))
    testing.expectf(t, size_of(PxSphereGeometry) == 12, "Wrong size for type PxSphereGeometry, expected 12 got %v", size_of(PxSphereGeometry))
}

@(test)
test_layout_PxPlaneGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPlaneGeometry) == 8, "Wrong size for type PxPlaneGeometry, expected 8 got %v", size_of(PxPlaneGeometry))
}

@(test)
test_layout_PxTriangleMeshGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTriangleMeshGeometry, scale) == 8, "Wrong offset for PxTriangleMeshGeometry.scale, expected 8 got %v", offset_of(PxTriangleMeshGeometry, scale))
    testing.expectf(t, offset_of(PxTriangleMeshGeometry, meshFlags) == 36, "Wrong offset for PxTriangleMeshGeometry.meshFlags, expected 36 got %v", offset_of(PxTriangleMeshGeometry, meshFlags))
    testing.expectf(t, offset_of(PxTriangleMeshGeometry, triangleMesh) == 40, "Wrong offset for PxTriangleMeshGeometry.triangleMesh, expected 40 got %v", offset_of(PxTriangleMeshGeometry, triangleMesh))
    testing.expectf(t, size_of(PxTriangleMeshGeometry) == 48, "Wrong size for type PxTriangleMeshGeometry, expected 48 got %v", size_of(PxTriangleMeshGeometry))
}

@(test)
test_layout_PxHeightFieldGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxHeightFieldGeometry, heightField) == 8, "Wrong offset for PxHeightFieldGeometry.heightField, expected 8 got %v", offset_of(PxHeightFieldGeometry, heightField))
    testing.expectf(t, offset_of(PxHeightFieldGeometry, heightScale) == 16, "Wrong offset for PxHeightFieldGeometry.heightScale, expected 16 got %v", offset_of(PxHeightFieldGeometry, heightScale))
    testing.expectf(t, offset_of(PxHeightFieldGeometry, rowScale) == 20, "Wrong offset for PxHeightFieldGeometry.rowScale, expected 20 got %v", offset_of(PxHeightFieldGeometry, rowScale))
    testing.expectf(t, offset_of(PxHeightFieldGeometry, columnScale) == 24, "Wrong offset for PxHeightFieldGeometry.columnScale, expected 24 got %v", offset_of(PxHeightFieldGeometry, columnScale))
    testing.expectf(t, offset_of(PxHeightFieldGeometry, heightFieldFlags) == 28, "Wrong offset for PxHeightFieldGeometry.heightFieldFlags, expected 28 got %v", offset_of(PxHeightFieldGeometry, heightFieldFlags))
    testing.expectf(t, size_of(PxHeightFieldGeometry) == 32, "Wrong size for type PxHeightFieldGeometry, expected 32 got %v", size_of(PxHeightFieldGeometry))
}

@(test)
test_layout_PxParticleSystemGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxParticleSystemGeometry, mSolverType) == 8, "Wrong offset for PxParticleSystemGeometry.mSolverType, expected 8 got %v", offset_of(PxParticleSystemGeometry, mSolverType))
    testing.expectf(t, size_of(PxParticleSystemGeometry) == 12, "Wrong size for type PxParticleSystemGeometry, expected 12 got %v", size_of(PxParticleSystemGeometry))
}

@(test)
test_layout_PxHairSystemGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxHairSystemGeometry) == 8, "Wrong size for type PxHairSystemGeometry, expected 8 got %v", size_of(PxHairSystemGeometry))
}

@(test)
test_layout_PxTetrahedronMeshGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTetrahedronMeshGeometry, tetrahedronMesh) == 8, "Wrong offset for PxTetrahedronMeshGeometry.tetrahedronMesh, expected 8 got %v", offset_of(PxTetrahedronMeshGeometry, tetrahedronMesh))
    testing.expectf(t, size_of(PxTetrahedronMeshGeometry) == 16, "Wrong size for type PxTetrahedronMeshGeometry, expected 16 got %v", size_of(PxTetrahedronMeshGeometry))
}

@(test)
test_layout_PxQueryHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxQueryHit) == 4, "Wrong size for type PxQueryHit, expected 4 got %v", size_of(PxQueryHit))
}

@(test)
test_layout_PxLocationHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxLocationHit, flags) == 4, "Wrong offset for PxLocationHit.flags, expected 4 got %v", offset_of(PxLocationHit, flags))
    testing.expectf(t, offset_of(PxLocationHit, position) == 8, "Wrong offset for PxLocationHit.position, expected 8 got %v", offset_of(PxLocationHit, position))
    testing.expectf(t, offset_of(PxLocationHit, normal) == 20, "Wrong offset for PxLocationHit.normal, expected 20 got %v", offset_of(PxLocationHit, normal))
    testing.expectf(t, offset_of(PxLocationHit, distance) == 32, "Wrong offset for PxLocationHit.distance, expected 32 got %v", offset_of(PxLocationHit, distance))
    testing.expectf(t, size_of(PxLocationHit) == 36, "Wrong size for type PxLocationHit, expected 36 got %v", size_of(PxLocationHit))
}

@(test)
test_layout_PxGeomRaycastHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGeomRaycastHit, u) == 36, "Wrong offset for PxGeomRaycastHit.u, expected 36 got %v", offset_of(PxGeomRaycastHit, u))
    testing.expectf(t, offset_of(PxGeomRaycastHit, v) == 40, "Wrong offset for PxGeomRaycastHit.v, expected 40 got %v", offset_of(PxGeomRaycastHit, v))
    testing.expectf(t, size_of(PxGeomRaycastHit) == 44, "Wrong size for type PxGeomRaycastHit, expected 44 got %v", size_of(PxGeomRaycastHit))
}

@(test)
test_layout_PxGeomOverlapHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxGeomOverlapHit) == 4, "Wrong size for type PxGeomOverlapHit, expected 4 got %v", size_of(PxGeomOverlapHit))
}

@(test)
test_layout_PxGeomSweepHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxGeomSweepHit) == 36, "Wrong size for type PxGeomSweepHit, expected 36 got %v", size_of(PxGeomSweepHit))
}

@(test)
test_layout_PxGeomIndexPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGeomIndexPair, id1) == 4, "Wrong offset for PxGeomIndexPair.id1, expected 4 got %v", offset_of(PxGeomIndexPair, id1))
    testing.expectf(t, size_of(PxGeomIndexPair) == 8, "Wrong size for type PxGeomIndexPair, expected 8 got %v", size_of(PxGeomIndexPair))
}


@(test)
test_layout_PxCustomGeometryType :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCustomGeometryType) == 4, "Wrong size for type PxCustomGeometryType, expected 4 got %v", size_of(PxCustomGeometryType))
}

@(test)
test_layout_PxCustomGeometryCallbacks :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCustomGeometryCallbacks) == 8, "Wrong size for type PxCustomGeometryCallbacks, expected 8 got %v", size_of(PxCustomGeometryCallbacks))
}

@(test)
test_layout_PxCustomGeometry :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxCustomGeometry, callbacks) == 8, "Wrong offset for PxCustomGeometry.callbacks, expected 8 got %v", offset_of(PxCustomGeometry, callbacks))
    testing.expectf(t, size_of(PxCustomGeometry) == 16, "Wrong size for type PxCustomGeometry, expected 16 got %v", size_of(PxCustomGeometry))
}

@(test)
test_layout_PxGeometryHolder :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxGeometryHolder) == 56, "Wrong size for type PxGeometryHolder, expected 56 got %v", size_of(PxGeometryHolder))
}


@(test)
test_layout_PxHeightFieldSample :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxHeightFieldSample, materialIndex0) == 2, "Wrong offset for PxHeightFieldSample.materialIndex0, expected 2 got %v", offset_of(PxHeightFieldSample, materialIndex0))
    testing.expectf(t, offset_of(PxHeightFieldSample, materialIndex1) == 3, "Wrong offset for PxHeightFieldSample.materialIndex1, expected 3 got %v", offset_of(PxHeightFieldSample, materialIndex1))
    testing.expectf(t, size_of(PxHeightFieldSample) == 4, "Wrong size for type PxHeightFieldSample, expected 4 got %v", size_of(PxHeightFieldSample))
}

@(test)
test_layout_PxHeightField :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxHeightField) == 16, "Wrong size for type PxHeightField, expected 16 got %v", size_of(PxHeightField))
}

@(test)
test_layout_PxHeightFieldDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxHeightFieldDesc, nbColumns) == 4, "Wrong offset for PxHeightFieldDesc.nbColumns, expected 4 got %v", offset_of(PxHeightFieldDesc, nbColumns))
    testing.expectf(t, offset_of(PxHeightFieldDesc, format) == 8, "Wrong offset for PxHeightFieldDesc.format, expected 8 got %v", offset_of(PxHeightFieldDesc, format))
    testing.expectf(t, offset_of(PxHeightFieldDesc, samples) == 16, "Wrong offset for PxHeightFieldDesc.samples, expected 16 got %v", offset_of(PxHeightFieldDesc, samples))
    testing.expectf(t, offset_of(PxHeightFieldDesc, convexEdgeThreshold) == 32, "Wrong offset for PxHeightFieldDesc.convexEdgeThreshold, expected 32 got %v", offset_of(PxHeightFieldDesc, convexEdgeThreshold))
    testing.expectf(t, offset_of(PxHeightFieldDesc, flags) == 36, "Wrong offset for PxHeightFieldDesc.flags, expected 36 got %v", offset_of(PxHeightFieldDesc, flags))
    testing.expectf(t, size_of(PxHeightFieldDesc) == 40, "Wrong size for type PxHeightFieldDesc, expected 40 got %v", size_of(PxHeightFieldDesc))
}


@(test)
test_layout_PxSimpleTriangleMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSimpleTriangleMesh, triangles) == 24, "Wrong offset for PxSimpleTriangleMesh.triangles, expected 24 got %v", offset_of(PxSimpleTriangleMesh, triangles))
    testing.expectf(t, offset_of(PxSimpleTriangleMesh, flags) == 48, "Wrong offset for PxSimpleTriangleMesh.flags, expected 48 got %v", offset_of(PxSimpleTriangleMesh, flags))
    testing.expectf(t, size_of(PxSimpleTriangleMesh) == 56, "Wrong size for type PxSimpleTriangleMesh, expected 56 got %v", size_of(PxSimpleTriangleMesh))
}

@(test)
test_layout_PxTriangle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTriangle) == 36, "Wrong size for type PxTriangle, expected 36 got %v", size_of(PxTriangle))
}

@(test)
test_layout_PxTrianglePadded :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTrianglePadded, padding) == 36, "Wrong offset for PxTrianglePadded.padding, expected 36 got %v", offset_of(PxTrianglePadded, padding))
    testing.expectf(t, size_of(PxTrianglePadded) == 40, "Wrong size for type PxTrianglePadded, expected 40 got %v", size_of(PxTrianglePadded))
}

@(test)
test_layout_PxTriangleMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTriangleMesh) == 16, "Wrong size for type PxTriangleMesh, expected 16 got %v", size_of(PxTriangleMesh))
}

@(test)
test_layout_PxBVH34TriangleMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBVH34TriangleMesh) == 16, "Wrong size for type PxBVH34TriangleMesh, expected 16 got %v", size_of(PxBVH34TriangleMesh))
}

@(test)
test_layout_PxTetrahedron :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTetrahedron) == 48, "Wrong size for type PxTetrahedron, expected 48 got %v", size_of(PxTetrahedron))
}

@(test)
test_layout_PxSoftBodyAuxData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSoftBodyAuxData) == 16, "Wrong size for type PxSoftBodyAuxData, expected 16 got %v", size_of(PxSoftBodyAuxData))
}

@(test)
test_layout_PxTetrahedronMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTetrahedronMesh) == 16, "Wrong size for type PxTetrahedronMesh, expected 16 got %v", size_of(PxTetrahedronMesh))
}

@(test)
test_layout_PxSoftBodyMesh :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSoftBodyMesh) == 16, "Wrong size for type PxSoftBodyMesh, expected 16 got %v", size_of(PxSoftBodyMesh))
}

@(test)
test_layout_PxCollisionMeshMappingData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCollisionMeshMappingData) == 8, "Wrong size for type PxCollisionMeshMappingData, expected 8 got %v", size_of(PxCollisionMeshMappingData))
}




@(test)
test_layout_PxCollisionTetrahedronMeshData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCollisionTetrahedronMeshData) == 8, "Wrong size for type PxCollisionTetrahedronMeshData, expected 8 got %v", size_of(PxCollisionTetrahedronMeshData))
}

@(test)
test_layout_PxSimulationTetrahedronMeshData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSimulationTetrahedronMeshData) == 8, "Wrong size for type PxSimulationTetrahedronMeshData, expected 8 got %v", size_of(PxSimulationTetrahedronMeshData))
}

@(test)
test_layout_PxActor :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxActor, userData) == 16, "Wrong offset for PxActor.userData, expected 16 got %v", offset_of(PxActor, userData))
    testing.expectf(t, size_of(PxActor) == 24, "Wrong size for type PxActor, expected 24 got %v", size_of(PxActor))
}

@(test)
test_layout_PxAggregate :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxAggregate, userData) == 16, "Wrong offset for PxAggregate.userData, expected 16 got %v", offset_of(PxAggregate, userData))
    testing.expectf(t, size_of(PxAggregate) == 24, "Wrong size for type PxAggregate, expected 24 got %v", size_of(PxAggregate))
}

@(test)
test_layout_PxSpringModifiers :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSpringModifiers, damping) == 4, "Wrong offset for PxSpringModifiers.damping, expected 4 got %v", offset_of(PxSpringModifiers, damping))
    testing.expectf(t, size_of(PxSpringModifiers) == 16, "Wrong size for type PxSpringModifiers, expected 16 got %v", size_of(PxSpringModifiers))
}

@(test)
test_layout_PxRestitutionModifiers :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxRestitutionModifiers, velocityThreshold) == 4, "Wrong offset for PxRestitutionModifiers.velocityThreshold, expected 4 got %v", offset_of(PxRestitutionModifiers, velocityThreshold))
    testing.expectf(t, size_of(PxRestitutionModifiers) == 16, "Wrong size for type PxRestitutionModifiers, expected 16 got %v", size_of(PxRestitutionModifiers))
}


@(test)
test_layout_Px1DConstraint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(Px1DConstraint, geometricError) == 12, "Wrong offset for Px1DConstraint.geometricError, expected 12 got %v", offset_of(Px1DConstraint, geometricError))
    testing.expectf(t, offset_of(Px1DConstraint, angular0) == 16, "Wrong offset for Px1DConstraint.angular0, expected 16 got %v", offset_of(Px1DConstraint, angular0))
    testing.expectf(t, offset_of(Px1DConstraint, velocityTarget) == 28, "Wrong offset for Px1DConstraint.velocityTarget, expected 28 got %v", offset_of(Px1DConstraint, velocityTarget))
    testing.expectf(t, offset_of(Px1DConstraint, linear1) == 32, "Wrong offset for Px1DConstraint.linear1, expected 32 got %v", offset_of(Px1DConstraint, linear1))
    testing.expectf(t, offset_of(Px1DConstraint, minImpulse) == 44, "Wrong offset for Px1DConstraint.minImpulse, expected 44 got %v", offset_of(Px1DConstraint, minImpulse))
    testing.expectf(t, offset_of(Px1DConstraint, angular1) == 48, "Wrong offset for Px1DConstraint.angular1, expected 48 got %v", offset_of(Px1DConstraint, angular1))
    testing.expectf(t, offset_of(Px1DConstraint, maxImpulse) == 60, "Wrong offset for Px1DConstraint.maxImpulse, expected 60 got %v", offset_of(Px1DConstraint, maxImpulse))
    testing.expectf(t, offset_of(Px1DConstraint, mods) == 64, "Wrong offset for Px1DConstraint.mods, expected 64 got %v", offset_of(Px1DConstraint, mods))
    testing.expectf(t, offset_of(Px1DConstraint, forInternalUse) == 80, "Wrong offset for Px1DConstraint.forInternalUse, expected 80 got %v", offset_of(Px1DConstraint, forInternalUse))
    testing.expectf(t, offset_of(Px1DConstraint, flags) == 84, "Wrong offset for Px1DConstraint.flags, expected 84 got %v", offset_of(Px1DConstraint, flags))
    testing.expectf(t, offset_of(Px1DConstraint, solveHint) == 86, "Wrong offset for Px1DConstraint.solveHint, expected 86 got %v", offset_of(Px1DConstraint, solveHint))
    testing.expectf(t, size_of(Px1DConstraint) == 96, "Wrong size for type Px1DConstraint, expected 96 got %v", size_of(Px1DConstraint))
}

@(test)
test_layout_PxConstraintInvMassScale :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConstraintInvMassScale, angular0) == 4, "Wrong offset for PxConstraintInvMassScale.angular0, expected 4 got %v", offset_of(PxConstraintInvMassScale, angular0))
    testing.expectf(t, offset_of(PxConstraintInvMassScale, linear1) == 8, "Wrong offset for PxConstraintInvMassScale.linear1, expected 8 got %v", offset_of(PxConstraintInvMassScale, linear1))
    testing.expectf(t, offset_of(PxConstraintInvMassScale, angular1) == 12, "Wrong offset for PxConstraintInvMassScale.angular1, expected 12 got %v", offset_of(PxConstraintInvMassScale, angular1))
    testing.expectf(t, size_of(PxConstraintInvMassScale) == 16, "Wrong size for type PxConstraintInvMassScale, expected 16 got %v", size_of(PxConstraintInvMassScale))
}

@(test)
test_layout_PxConstraintVisualizer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxConstraintVisualizer) == 8, "Wrong size for type PxConstraintVisualizer, expected 8 got %v", size_of(PxConstraintVisualizer))
}

@(test)
test_layout_PxConstraintConnector :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxConstraintConnector) == 8, "Wrong size for type PxConstraintConnector, expected 8 got %v", size_of(PxConstraintConnector))
}

@(test)
test_layout_PxContactPoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPoint, separation) == 12, "Wrong offset for PxContactPoint.separation, expected 12 got %v", offset_of(PxContactPoint, separation))
    testing.expectf(t, offset_of(PxContactPoint, point) == 16, "Wrong offset for PxContactPoint.point, expected 16 got %v", offset_of(PxContactPoint, point))
    testing.expectf(t, offset_of(PxContactPoint, maxImpulse) == 28, "Wrong offset for PxContactPoint.maxImpulse, expected 28 got %v", offset_of(PxContactPoint, maxImpulse))
    testing.expectf(t, offset_of(PxContactPoint, targetVel) == 32, "Wrong offset for PxContactPoint.targetVel, expected 32 got %v", offset_of(PxContactPoint, targetVel))
    testing.expectf(t, offset_of(PxContactPoint, staticFriction) == 44, "Wrong offset for PxContactPoint.staticFriction, expected 44 got %v", offset_of(PxContactPoint, staticFriction))
    testing.expectf(t, offset_of(PxContactPoint, materialFlags) == 48, "Wrong offset for PxContactPoint.materialFlags, expected 48 got %v", offset_of(PxContactPoint, materialFlags))
    testing.expectf(t, offset_of(PxContactPoint, internalFaceIndex1) == 52, "Wrong offset for PxContactPoint.internalFaceIndex1, expected 52 got %v", offset_of(PxContactPoint, internalFaceIndex1))
    testing.expectf(t, offset_of(PxContactPoint, dynamicFriction) == 56, "Wrong offset for PxContactPoint.dynamicFriction, expected 56 got %v", offset_of(PxContactPoint, dynamicFriction))
    testing.expectf(t, offset_of(PxContactPoint, restitution) == 60, "Wrong offset for PxContactPoint.restitution, expected 60 got %v", offset_of(PxContactPoint, restitution))
    testing.expectf(t, offset_of(PxContactPoint, damping) == 64, "Wrong offset for PxContactPoint.damping, expected 64 got %v", offset_of(PxContactPoint, damping))
    testing.expectf(t, size_of(PxContactPoint) == 80, "Wrong size for type PxContactPoint, expected 80 got %v", size_of(PxContactPoint))
}

@(test)
test_layout_PxSolverBody :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSolverBody, maxSolverNormalProgress) == 12, "Wrong offset for PxSolverBody.maxSolverNormalProgress, expected 12 got %v", offset_of(PxSolverBody, maxSolverNormalProgress))
    testing.expectf(t, offset_of(PxSolverBody, maxSolverFrictionProgress) == 14, "Wrong offset for PxSolverBody.maxSolverFrictionProgress, expected 14 got %v", offset_of(PxSolverBody, maxSolverFrictionProgress))
    testing.expectf(t, offset_of(PxSolverBody, angularState) == 16, "Wrong offset for PxSolverBody.angularState, expected 16 got %v", offset_of(PxSolverBody, angularState))
    testing.expectf(t, offset_of(PxSolverBody, solverProgress) == 28, "Wrong offset for PxSolverBody.solverProgress, expected 28 got %v", offset_of(PxSolverBody, solverProgress))
    testing.expectf(t, size_of(PxSolverBody) == 32, "Wrong size for type PxSolverBody, expected 32 got %v", size_of(PxSolverBody))
}

@(test)
test_layout_PxSolverBodyData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSolverBodyData, invMass) == 12, "Wrong offset for PxSolverBodyData.invMass, expected 12 got %v", offset_of(PxSolverBodyData, invMass))
    testing.expectf(t, offset_of(PxSolverBodyData, angularVelocity) == 16, "Wrong offset for PxSolverBodyData.angularVelocity, expected 16 got %v", offset_of(PxSolverBodyData, angularVelocity))
    testing.expectf(t, offset_of(PxSolverBodyData, reportThreshold) == 28, "Wrong offset for PxSolverBodyData.reportThreshold, expected 28 got %v", offset_of(PxSolverBodyData, reportThreshold))
    testing.expectf(t, offset_of(PxSolverBodyData, sqrtInvInertia) == 32, "Wrong offset for PxSolverBodyData.sqrtInvInertia, expected 32 got %v", offset_of(PxSolverBodyData, sqrtInvInertia))
    testing.expectf(t, offset_of(PxSolverBodyData, penBiasClamp) == 68, "Wrong offset for PxSolverBodyData.penBiasClamp, expected 68 got %v", offset_of(PxSolverBodyData, penBiasClamp))
    testing.expectf(t, offset_of(PxSolverBodyData, nodeIndex) == 72, "Wrong offset for PxSolverBodyData.nodeIndex, expected 72 got %v", offset_of(PxSolverBodyData, nodeIndex))
    testing.expectf(t, offset_of(PxSolverBodyData, maxContactImpulse) == 76, "Wrong offset for PxSolverBodyData.maxContactImpulse, expected 76 got %v", offset_of(PxSolverBodyData, maxContactImpulse))
    testing.expectf(t, offset_of(PxSolverBodyData, body2World) == 80, "Wrong offset for PxSolverBodyData.body2World, expected 80 got %v", offset_of(PxSolverBodyData, body2World))
    testing.expectf(t, offset_of(PxSolverBodyData, pad) == 108, "Wrong offset for PxSolverBodyData.pad, expected 108 got %v", offset_of(PxSolverBodyData, pad))
    testing.expectf(t, size_of(PxSolverBodyData) == 112, "Wrong size for type PxSolverBodyData, expected 112 got %v", size_of(PxSolverBodyData))
}

@(test)
test_layout_PxConstraintBatchHeader :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConstraintBatchHeader, stride) == 4, "Wrong offset for PxConstraintBatchHeader.stride, expected 4 got %v", offset_of(PxConstraintBatchHeader, stride))
    testing.expectf(t, offset_of(PxConstraintBatchHeader, constraintType) == 6, "Wrong offset for PxConstraintBatchHeader.constraintType, expected 6 got %v", offset_of(PxConstraintBatchHeader, constraintType))
    testing.expectf(t, size_of(PxConstraintBatchHeader) == 8, "Wrong size for type PxConstraintBatchHeader, expected 8 got %v", size_of(PxConstraintBatchHeader))
}

@(test)
test_layout_PxSolverConstraintDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSolverConstraintDesc, bodyB) == 8, "Wrong offset for PxSolverConstraintDesc.bodyB, expected 8 got %v", offset_of(PxSolverConstraintDesc, bodyB))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, bodyADataIndex) == 16, "Wrong offset for PxSolverConstraintDesc.bodyADataIndex, expected 16 got %v", offset_of(PxSolverConstraintDesc, bodyADataIndex))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, bodyBDataIndex) == 20, "Wrong offset for PxSolverConstraintDesc.bodyBDataIndex, expected 20 got %v", offset_of(PxSolverConstraintDesc, bodyBDataIndex))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, linkIndexA) == 24, "Wrong offset for PxSolverConstraintDesc.linkIndexA, expected 24 got %v", offset_of(PxSolverConstraintDesc, linkIndexA))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, linkIndexB) == 28, "Wrong offset for PxSolverConstraintDesc.linkIndexB, expected 28 got %v", offset_of(PxSolverConstraintDesc, linkIndexB))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, constraint) == 32, "Wrong offset for PxSolverConstraintDesc.constraint, expected 32 got %v", offset_of(PxSolverConstraintDesc, constraint))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, writeBack) == 40, "Wrong offset for PxSolverConstraintDesc.writeBack, expected 40 got %v", offset_of(PxSolverConstraintDesc, writeBack))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, progressA) == 48, "Wrong offset for PxSolverConstraintDesc.progressA, expected 48 got %v", offset_of(PxSolverConstraintDesc, progressA))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, progressB) == 50, "Wrong offset for PxSolverConstraintDesc.progressB, expected 50 got %v", offset_of(PxSolverConstraintDesc, progressB))
    testing.expectf(t, offset_of(PxSolverConstraintDesc, constraintLengthOver16) == 52, "Wrong offset for PxSolverConstraintDesc.constraintLengthOver16, expected 52 got %v", offset_of(PxSolverConstraintDesc, constraintLengthOver16))
    testing.expectf(t, size_of(PxSolverConstraintDesc) == 64, "Wrong size for type PxSolverConstraintDesc, expected 64 got %v", size_of(PxSolverConstraintDesc))
}

@(test)
test_layout_PxSolverConstraintPrepDescBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, desc) == 16, "Wrong offset for PxSolverConstraintPrepDescBase.desc, expected 16 got %v", offset_of(PxSolverConstraintPrepDescBase, desc))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, body0) == 24, "Wrong offset for PxSolverConstraintPrepDescBase.body0, expected 24 got %v", offset_of(PxSolverConstraintPrepDescBase, body0))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, body1) == 32, "Wrong offset for PxSolverConstraintPrepDescBase.body1, expected 32 got %v", offset_of(PxSolverConstraintPrepDescBase, body1))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, data0) == 40, "Wrong offset for PxSolverConstraintPrepDescBase.data0, expected 40 got %v", offset_of(PxSolverConstraintPrepDescBase, data0))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, data1) == 48, "Wrong offset for PxSolverConstraintPrepDescBase.data1, expected 48 got %v", offset_of(PxSolverConstraintPrepDescBase, data1))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, bodyFrame0) == 56, "Wrong offset for PxSolverConstraintPrepDescBase.bodyFrame0, expected 56 got %v", offset_of(PxSolverConstraintPrepDescBase, bodyFrame0))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, bodyFrame1) == 84, "Wrong offset for PxSolverConstraintPrepDescBase.bodyFrame1, expected 84 got %v", offset_of(PxSolverConstraintPrepDescBase, bodyFrame1))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, bodyState0) == 112, "Wrong offset for PxSolverConstraintPrepDescBase.bodyState0, expected 112 got %v", offset_of(PxSolverConstraintPrepDescBase, bodyState0))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDescBase, bodyState1) == 116, "Wrong offset for PxSolverConstraintPrepDescBase.bodyState1, expected 116 got %v", offset_of(PxSolverConstraintPrepDescBase, bodyState1))
    testing.expectf(t, size_of(PxSolverConstraintPrepDescBase) == 128, "Wrong size for type PxSolverConstraintPrepDescBase, expected 128 got %v", size_of(PxSolverConstraintPrepDescBase))
}

@(test)
test_layout_PxSolverConstraintPrepDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, rows) == 128, "Wrong offset for PxSolverConstraintPrepDesc.rows, expected 128 got %v", offset_of(PxSolverConstraintPrepDesc, rows))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, numRows) == 136, "Wrong offset for PxSolverConstraintPrepDesc.numRows, expected 136 got %v", offset_of(PxSolverConstraintPrepDesc, numRows))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, linBreakForce) == 140, "Wrong offset for PxSolverConstraintPrepDesc.linBreakForce, expected 140 got %v", offset_of(PxSolverConstraintPrepDesc, linBreakForce))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, angBreakForce) == 144, "Wrong offset for PxSolverConstraintPrepDesc.angBreakForce, expected 144 got %v", offset_of(PxSolverConstraintPrepDesc, angBreakForce))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, minResponseThreshold) == 148, "Wrong offset for PxSolverConstraintPrepDesc.minResponseThreshold, expected 148 got %v", offset_of(PxSolverConstraintPrepDesc, minResponseThreshold))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, writeback) == 152, "Wrong offset for PxSolverConstraintPrepDesc.writeback, expected 152 got %v", offset_of(PxSolverConstraintPrepDesc, writeback))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, disablePreprocessing) == 160, "Wrong offset for PxSolverConstraintPrepDesc.disablePreprocessing, expected 160 got %v", offset_of(PxSolverConstraintPrepDesc, disablePreprocessing))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, improvedSlerp) == 161, "Wrong offset for PxSolverConstraintPrepDesc.improvedSlerp, expected 161 got %v", offset_of(PxSolverConstraintPrepDesc, improvedSlerp))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, driveLimitsAreForces) == 162, "Wrong offset for PxSolverConstraintPrepDesc.driveLimitsAreForces, expected 162 got %v", offset_of(PxSolverConstraintPrepDesc, driveLimitsAreForces))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, extendedLimits) == 163, "Wrong offset for PxSolverConstraintPrepDesc.extendedLimits, expected 163 got %v", offset_of(PxSolverConstraintPrepDesc, extendedLimits))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, disableConstraint) == 164, "Wrong offset for PxSolverConstraintPrepDesc.disableConstraint, expected 164 got %v", offset_of(PxSolverConstraintPrepDesc, disableConstraint))
    testing.expectf(t, offset_of(PxSolverConstraintPrepDesc, body0WorldOffset) == 168, "Wrong offset for PxSolverConstraintPrepDesc.body0WorldOffset, expected 168 got %v", offset_of(PxSolverConstraintPrepDesc, body0WorldOffset))
    testing.expectf(t, size_of(PxSolverConstraintPrepDesc) == 192, "Wrong size for type PxSolverConstraintPrepDesc, expected 192 got %v", size_of(PxSolverConstraintPrepDesc))
}

@(test)
test_layout_PxSolverContactDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSolverContactDesc, desc) == 16, "Wrong offset for PxSolverContactDesc.desc, expected 16 got %v", offset_of(PxSolverContactDesc, desc))
    testing.expectf(t, offset_of(PxSolverContactDesc, body0) == 24, "Wrong offset for PxSolverContactDesc.body0, expected 24 got %v", offset_of(PxSolverContactDesc, body0))
    testing.expectf(t, offset_of(PxSolverContactDesc, body1) == 32, "Wrong offset for PxSolverContactDesc.body1, expected 32 got %v", offset_of(PxSolverContactDesc, body1))
    testing.expectf(t, offset_of(PxSolverContactDesc, data0) == 40, "Wrong offset for PxSolverContactDesc.data0, expected 40 got %v", offset_of(PxSolverContactDesc, data0))
    testing.expectf(t, offset_of(PxSolverContactDesc, data1) == 48, "Wrong offset for PxSolverContactDesc.data1, expected 48 got %v", offset_of(PxSolverContactDesc, data1))
    testing.expectf(t, offset_of(PxSolverContactDesc, bodyFrame0) == 56, "Wrong offset for PxSolverContactDesc.bodyFrame0, expected 56 got %v", offset_of(PxSolverContactDesc, bodyFrame0))
    testing.expectf(t, offset_of(PxSolverContactDesc, bodyFrame1) == 84, "Wrong offset for PxSolverContactDesc.bodyFrame1, expected 84 got %v", offset_of(PxSolverContactDesc, bodyFrame1))
    testing.expectf(t, offset_of(PxSolverContactDesc, bodyState0) == 112, "Wrong offset for PxSolverContactDesc.bodyState0, expected 112 got %v", offset_of(PxSolverContactDesc, bodyState0))
    testing.expectf(t, offset_of(PxSolverContactDesc, bodyState1) == 116, "Wrong offset for PxSolverContactDesc.bodyState1, expected 116 got %v", offset_of(PxSolverContactDesc, bodyState1))
    testing.expectf(t, offset_of(PxSolverContactDesc, shapeInteraction) == 120, "Wrong offset for PxSolverContactDesc.shapeInteraction, expected 120 got %v", offset_of(PxSolverContactDesc, shapeInteraction))
    testing.expectf(t, offset_of(PxSolverContactDesc, contacts) == 128, "Wrong offset for PxSolverContactDesc.contacts, expected 128 got %v", offset_of(PxSolverContactDesc, contacts))
    testing.expectf(t, offset_of(PxSolverContactDesc, numContacts) == 136, "Wrong offset for PxSolverContactDesc.numContacts, expected 136 got %v", offset_of(PxSolverContactDesc, numContacts))
    testing.expectf(t, offset_of(PxSolverContactDesc, hasMaxImpulse) == 140, "Wrong offset for PxSolverContactDesc.hasMaxImpulse, expected 140 got %v", offset_of(PxSolverContactDesc, hasMaxImpulse))
    testing.expectf(t, offset_of(PxSolverContactDesc, disableStrongFriction) == 141, "Wrong offset for PxSolverContactDesc.disableStrongFriction, expected 141 got %v", offset_of(PxSolverContactDesc, disableStrongFriction))
    testing.expectf(t, offset_of(PxSolverContactDesc, hasForceThresholds) == 142, "Wrong offset for PxSolverContactDesc.hasForceThresholds, expected 142 got %v", offset_of(PxSolverContactDesc, hasForceThresholds))
    testing.expectf(t, offset_of(PxSolverContactDesc, restDistance) == 144, "Wrong offset for PxSolverContactDesc.restDistance, expected 144 got %v", offset_of(PxSolverContactDesc, restDistance))
    testing.expectf(t, offset_of(PxSolverContactDesc, maxCCDSeparation) == 148, "Wrong offset for PxSolverContactDesc.maxCCDSeparation, expected 148 got %v", offset_of(PxSolverContactDesc, maxCCDSeparation))
    testing.expectf(t, offset_of(PxSolverContactDesc, frictionPtr) == 152, "Wrong offset for PxSolverContactDesc.frictionPtr, expected 152 got %v", offset_of(PxSolverContactDesc, frictionPtr))
    testing.expectf(t, offset_of(PxSolverContactDesc, frictionCount) == 160, "Wrong offset for PxSolverContactDesc.frictionCount, expected 160 got %v", offset_of(PxSolverContactDesc, frictionCount))
    testing.expectf(t, offset_of(PxSolverContactDesc, contactForces) == 168, "Wrong offset for PxSolverContactDesc.contactForces, expected 168 got %v", offset_of(PxSolverContactDesc, contactForces))
    testing.expectf(t, offset_of(PxSolverContactDesc, startFrictionPatchIndex) == 176, "Wrong offset for PxSolverContactDesc.startFrictionPatchIndex, expected 176 got %v", offset_of(PxSolverContactDesc, startFrictionPatchIndex))
    testing.expectf(t, offset_of(PxSolverContactDesc, numFrictionPatches) == 180, "Wrong offset for PxSolverContactDesc.numFrictionPatches, expected 180 got %v", offset_of(PxSolverContactDesc, numFrictionPatches))
    testing.expectf(t, offset_of(PxSolverContactDesc, startContactPatchIndex) == 184, "Wrong offset for PxSolverContactDesc.startContactPatchIndex, expected 184 got %v", offset_of(PxSolverContactDesc, startContactPatchIndex))
    testing.expectf(t, offset_of(PxSolverContactDesc, numContactPatches) == 188, "Wrong offset for PxSolverContactDesc.numContactPatches, expected 188 got %v", offset_of(PxSolverContactDesc, numContactPatches))
    testing.expectf(t, offset_of(PxSolverContactDesc, axisConstraintCount) == 190, "Wrong offset for PxSolverContactDesc.axisConstraintCount, expected 190 got %v", offset_of(PxSolverContactDesc, axisConstraintCount))
    testing.expectf(t, offset_of(PxSolverContactDesc, offsetSlop) == 192, "Wrong offset for PxSolverContactDesc.offsetSlop, expected 192 got %v", offset_of(PxSolverContactDesc, offsetSlop))
    testing.expectf(t, size_of(PxSolverContactDesc) == 208, "Wrong size for type PxSolverContactDesc, expected 208 got %v", size_of(PxSolverContactDesc))
}

@(test)
test_layout_PxConstraintAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxConstraintAllocator) == 8, "Wrong size for type PxConstraintAllocator, expected 8 got %v", size_of(PxConstraintAllocator))
}

@(test)
test_layout_PxArticulationLimit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationLimit, high) == 4, "Wrong offset for PxArticulationLimit.high, expected 4 got %v", offset_of(PxArticulationLimit, high))
    testing.expectf(t, size_of(PxArticulationLimit) == 8, "Wrong size for type PxArticulationLimit, expected 8 got %v", size_of(PxArticulationLimit))
}

@(test)
test_layout_PxArticulationDrive :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationDrive, damping) == 4, "Wrong offset for PxArticulationDrive.damping, expected 4 got %v", offset_of(PxArticulationDrive, damping))
    testing.expectf(t, offset_of(PxArticulationDrive, maxForce) == 8, "Wrong offset for PxArticulationDrive.maxForce, expected 8 got %v", offset_of(PxArticulationDrive, maxForce))
    testing.expectf(t, offset_of(PxArticulationDrive, driveType) == 12, "Wrong offset for PxArticulationDrive.driveType, expected 12 got %v", offset_of(PxArticulationDrive, driveType))
    testing.expectf(t, size_of(PxArticulationDrive) == 16, "Wrong size for type PxArticulationDrive, expected 16 got %v", size_of(PxArticulationDrive))
}

@(test)
test_layout_PxTGSSolverBodyVel :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, nbStaticInteractions) == 12, "Wrong offset for PxTGSSolverBodyVel.nbStaticInteractions, expected 12 got %v", offset_of(PxTGSSolverBodyVel, nbStaticInteractions))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, maxDynamicPartition) == 14, "Wrong offset for PxTGSSolverBodyVel.maxDynamicPartition, expected 14 got %v", offset_of(PxTGSSolverBodyVel, maxDynamicPartition))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, angularVelocity) == 16, "Wrong offset for PxTGSSolverBodyVel.angularVelocity, expected 16 got %v", offset_of(PxTGSSolverBodyVel, angularVelocity))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, partitionMask) == 28, "Wrong offset for PxTGSSolverBodyVel.partitionMask, expected 28 got %v", offset_of(PxTGSSolverBodyVel, partitionMask))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, deltaAngDt) == 32, "Wrong offset for PxTGSSolverBodyVel.deltaAngDt, expected 32 got %v", offset_of(PxTGSSolverBodyVel, deltaAngDt))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, maxAngVel) == 44, "Wrong offset for PxTGSSolverBodyVel.maxAngVel, expected 44 got %v", offset_of(PxTGSSolverBodyVel, maxAngVel))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, deltaLinDt) == 48, "Wrong offset for PxTGSSolverBodyVel.deltaLinDt, expected 48 got %v", offset_of(PxTGSSolverBodyVel, deltaLinDt))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, lockFlags) == 60, "Wrong offset for PxTGSSolverBodyVel.lockFlags, expected 60 got %v", offset_of(PxTGSSolverBodyVel, lockFlags))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, isKinematic) == 62, "Wrong offset for PxTGSSolverBodyVel.isKinematic, expected 62 got %v", offset_of(PxTGSSolverBodyVel, isKinematic))
    testing.expectf(t, offset_of(PxTGSSolverBodyVel, pad) == 63, "Wrong offset for PxTGSSolverBodyVel.pad, expected 63 got %v", offset_of(PxTGSSolverBodyVel, pad))
    testing.expectf(t, size_of(PxTGSSolverBodyVel) == 64, "Wrong size for type PxTGSSolverBodyVel, expected 64 got %v", size_of(PxTGSSolverBodyVel))
}

@(test)
test_layout_PxTGSSolverBodyTxInertia :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTGSSolverBodyTxInertia, sqrtInvInertia) == 28, "Wrong offset for PxTGSSolverBodyTxInertia.sqrtInvInertia, expected 28 got %v", offset_of(PxTGSSolverBodyTxInertia, sqrtInvInertia))
    testing.expectf(t, size_of(PxTGSSolverBodyTxInertia) == 64, "Wrong size for type PxTGSSolverBodyTxInertia, expected 64 got %v", size_of(PxTGSSolverBodyTxInertia))
}

@(test)
test_layout_PxTGSSolverBodyData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTGSSolverBodyData, maxContactImpulse) == 12, "Wrong offset for PxTGSSolverBodyData.maxContactImpulse, expected 12 got %v", offset_of(PxTGSSolverBodyData, maxContactImpulse))
    testing.expectf(t, offset_of(PxTGSSolverBodyData, originalAngularVelocity) == 16, "Wrong offset for PxTGSSolverBodyData.originalAngularVelocity, expected 16 got %v", offset_of(PxTGSSolverBodyData, originalAngularVelocity))
    testing.expectf(t, offset_of(PxTGSSolverBodyData, penBiasClamp) == 28, "Wrong offset for PxTGSSolverBodyData.penBiasClamp, expected 28 got %v", offset_of(PxTGSSolverBodyData, penBiasClamp))
    testing.expectf(t, offset_of(PxTGSSolverBodyData, invMass) == 32, "Wrong offset for PxTGSSolverBodyData.invMass, expected 32 got %v", offset_of(PxTGSSolverBodyData, invMass))
    testing.expectf(t, offset_of(PxTGSSolverBodyData, nodeIndex) == 36, "Wrong offset for PxTGSSolverBodyData.nodeIndex, expected 36 got %v", offset_of(PxTGSSolverBodyData, nodeIndex))
    testing.expectf(t, offset_of(PxTGSSolverBodyData, reportThreshold) == 40, "Wrong offset for PxTGSSolverBodyData.reportThreshold, expected 40 got %v", offset_of(PxTGSSolverBodyData, reportThreshold))
    testing.expectf(t, offset_of(PxTGSSolverBodyData, pad) == 44, "Wrong offset for PxTGSSolverBodyData.pad, expected 44 got %v", offset_of(PxTGSSolverBodyData, pad))
    testing.expectf(t, size_of(PxTGSSolverBodyData) == 48, "Wrong size for type PxTGSSolverBodyData, expected 48 got %v", size_of(PxTGSSolverBodyData))
}

@(test)
test_layout_PxTGSSolverConstraintPrepDescBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, desc) == 16, "Wrong offset for PxTGSSolverConstraintPrepDescBase.desc, expected 16 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, desc))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, body0) == 24, "Wrong offset for PxTGSSolverConstraintPrepDescBase.body0, expected 24 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, body0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, body1) == 32, "Wrong offset for PxTGSSolverConstraintPrepDescBase.body1, expected 32 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, body1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, body0TxI) == 40, "Wrong offset for PxTGSSolverConstraintPrepDescBase.body0TxI, expected 40 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, body0TxI))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, body1TxI) == 48, "Wrong offset for PxTGSSolverConstraintPrepDescBase.body1TxI, expected 48 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, body1TxI))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, bodyData0) == 56, "Wrong offset for PxTGSSolverConstraintPrepDescBase.bodyData0, expected 56 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, bodyData0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, bodyData1) == 64, "Wrong offset for PxTGSSolverConstraintPrepDescBase.bodyData1, expected 64 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, bodyData1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, bodyFrame0) == 72, "Wrong offset for PxTGSSolverConstraintPrepDescBase.bodyFrame0, expected 72 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, bodyFrame0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, bodyFrame1) == 100, "Wrong offset for PxTGSSolverConstraintPrepDescBase.bodyFrame1, expected 100 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, bodyFrame1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, bodyState0) == 128, "Wrong offset for PxTGSSolverConstraintPrepDescBase.bodyState0, expected 128 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, bodyState0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDescBase, bodyState1) == 132, "Wrong offset for PxTGSSolverConstraintPrepDescBase.bodyState1, expected 132 got %v", offset_of(PxTGSSolverConstraintPrepDescBase, bodyState1))
    testing.expectf(t, size_of(PxTGSSolverConstraintPrepDescBase) == 144, "Wrong size for type PxTGSSolverConstraintPrepDescBase, expected 144 got %v", size_of(PxTGSSolverConstraintPrepDescBase))
}

@(test)
test_layout_PxTGSSolverConstraintPrepDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, desc) == 16, "Wrong offset for PxTGSSolverConstraintPrepDesc.desc, expected 16 got %v", offset_of(PxTGSSolverConstraintPrepDesc, desc))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, body0) == 24, "Wrong offset for PxTGSSolverConstraintPrepDesc.body0, expected 24 got %v", offset_of(PxTGSSolverConstraintPrepDesc, body0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, body1) == 32, "Wrong offset for PxTGSSolverConstraintPrepDesc.body1, expected 32 got %v", offset_of(PxTGSSolverConstraintPrepDesc, body1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, body0TxI) == 40, "Wrong offset for PxTGSSolverConstraintPrepDesc.body0TxI, expected 40 got %v", offset_of(PxTGSSolverConstraintPrepDesc, body0TxI))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, body1TxI) == 48, "Wrong offset for PxTGSSolverConstraintPrepDesc.body1TxI, expected 48 got %v", offset_of(PxTGSSolverConstraintPrepDesc, body1TxI))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, bodyData0) == 56, "Wrong offset for PxTGSSolverConstraintPrepDesc.bodyData0, expected 56 got %v", offset_of(PxTGSSolverConstraintPrepDesc, bodyData0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, bodyData1) == 64, "Wrong offset for PxTGSSolverConstraintPrepDesc.bodyData1, expected 64 got %v", offset_of(PxTGSSolverConstraintPrepDesc, bodyData1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, bodyFrame0) == 72, "Wrong offset for PxTGSSolverConstraintPrepDesc.bodyFrame0, expected 72 got %v", offset_of(PxTGSSolverConstraintPrepDesc, bodyFrame0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, bodyFrame1) == 100, "Wrong offset for PxTGSSolverConstraintPrepDesc.bodyFrame1, expected 100 got %v", offset_of(PxTGSSolverConstraintPrepDesc, bodyFrame1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, bodyState0) == 128, "Wrong offset for PxTGSSolverConstraintPrepDesc.bodyState0, expected 128 got %v", offset_of(PxTGSSolverConstraintPrepDesc, bodyState0))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, bodyState1) == 132, "Wrong offset for PxTGSSolverConstraintPrepDesc.bodyState1, expected 132 got %v", offset_of(PxTGSSolverConstraintPrepDesc, bodyState1))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, rows) == 136, "Wrong offset for PxTGSSolverConstraintPrepDesc.rows, expected 136 got %v", offset_of(PxTGSSolverConstraintPrepDesc, rows))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, numRows) == 144, "Wrong offset for PxTGSSolverConstraintPrepDesc.numRows, expected 144 got %v", offset_of(PxTGSSolverConstraintPrepDesc, numRows))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, linBreakForce) == 148, "Wrong offset for PxTGSSolverConstraintPrepDesc.linBreakForce, expected 148 got %v", offset_of(PxTGSSolverConstraintPrepDesc, linBreakForce))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, angBreakForce) == 152, "Wrong offset for PxTGSSolverConstraintPrepDesc.angBreakForce, expected 152 got %v", offset_of(PxTGSSolverConstraintPrepDesc, angBreakForce))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, minResponseThreshold) == 156, "Wrong offset for PxTGSSolverConstraintPrepDesc.minResponseThreshold, expected 156 got %v", offset_of(PxTGSSolverConstraintPrepDesc, minResponseThreshold))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, writeback) == 160, "Wrong offset for PxTGSSolverConstraintPrepDesc.writeback, expected 160 got %v", offset_of(PxTGSSolverConstraintPrepDesc, writeback))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, disablePreprocessing) == 168, "Wrong offset for PxTGSSolverConstraintPrepDesc.disablePreprocessing, expected 168 got %v", offset_of(PxTGSSolverConstraintPrepDesc, disablePreprocessing))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, improvedSlerp) == 169, "Wrong offset for PxTGSSolverConstraintPrepDesc.improvedSlerp, expected 169 got %v", offset_of(PxTGSSolverConstraintPrepDesc, improvedSlerp))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, driveLimitsAreForces) == 170, "Wrong offset for PxTGSSolverConstraintPrepDesc.driveLimitsAreForces, expected 170 got %v", offset_of(PxTGSSolverConstraintPrepDesc, driveLimitsAreForces))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, extendedLimits) == 171, "Wrong offset for PxTGSSolverConstraintPrepDesc.extendedLimits, expected 171 got %v", offset_of(PxTGSSolverConstraintPrepDesc, extendedLimits))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, disableConstraint) == 172, "Wrong offset for PxTGSSolverConstraintPrepDesc.disableConstraint, expected 172 got %v", offset_of(PxTGSSolverConstraintPrepDesc, disableConstraint))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, body0WorldOffset) == 176, "Wrong offset for PxTGSSolverConstraintPrepDesc.body0WorldOffset, expected 176 got %v", offset_of(PxTGSSolverConstraintPrepDesc, body0WorldOffset))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, cA2w) == 192, "Wrong offset for PxTGSSolverConstraintPrepDesc.cA2w, expected 192 got %v", offset_of(PxTGSSolverConstraintPrepDesc, cA2w))
    testing.expectf(t, offset_of(PxTGSSolverConstraintPrepDesc, cB2w) == 208, "Wrong offset for PxTGSSolverConstraintPrepDesc.cB2w, expected 208 got %v", offset_of(PxTGSSolverConstraintPrepDesc, cB2w))
    testing.expectf(t, size_of(PxTGSSolverConstraintPrepDesc) == 224, "Wrong size for type PxTGSSolverConstraintPrepDesc, expected 224 got %v", size_of(PxTGSSolverConstraintPrepDesc))
}

@(test)
test_layout_PxTGSSolverContactDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, desc) == 16, "Wrong offset for PxTGSSolverContactDesc.desc, expected 16 got %v", offset_of(PxTGSSolverContactDesc, desc))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, body0) == 24, "Wrong offset for PxTGSSolverContactDesc.body0, expected 24 got %v", offset_of(PxTGSSolverContactDesc, body0))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, body1) == 32, "Wrong offset for PxTGSSolverContactDesc.body1, expected 32 got %v", offset_of(PxTGSSolverContactDesc, body1))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, body0TxI) == 40, "Wrong offset for PxTGSSolverContactDesc.body0TxI, expected 40 got %v", offset_of(PxTGSSolverContactDesc, body0TxI))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, body1TxI) == 48, "Wrong offset for PxTGSSolverContactDesc.body1TxI, expected 48 got %v", offset_of(PxTGSSolverContactDesc, body1TxI))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, bodyData0) == 56, "Wrong offset for PxTGSSolverContactDesc.bodyData0, expected 56 got %v", offset_of(PxTGSSolverContactDesc, bodyData0))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, bodyData1) == 64, "Wrong offset for PxTGSSolverContactDesc.bodyData1, expected 64 got %v", offset_of(PxTGSSolverContactDesc, bodyData1))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, bodyFrame0) == 72, "Wrong offset for PxTGSSolverContactDesc.bodyFrame0, expected 72 got %v", offset_of(PxTGSSolverContactDesc, bodyFrame0))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, bodyFrame1) == 100, "Wrong offset for PxTGSSolverContactDesc.bodyFrame1, expected 100 got %v", offset_of(PxTGSSolverContactDesc, bodyFrame1))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, bodyState0) == 128, "Wrong offset for PxTGSSolverContactDesc.bodyState0, expected 128 got %v", offset_of(PxTGSSolverContactDesc, bodyState0))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, bodyState1) == 132, "Wrong offset for PxTGSSolverContactDesc.bodyState1, expected 132 got %v", offset_of(PxTGSSolverContactDesc, bodyState1))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, shapeInteraction) == 136, "Wrong offset for PxTGSSolverContactDesc.shapeInteraction, expected 136 got %v", offset_of(PxTGSSolverContactDesc, shapeInteraction))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, contacts) == 144, "Wrong offset for PxTGSSolverContactDesc.contacts, expected 144 got %v", offset_of(PxTGSSolverContactDesc, contacts))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, numContacts) == 152, "Wrong offset for PxTGSSolverContactDesc.numContacts, expected 152 got %v", offset_of(PxTGSSolverContactDesc, numContacts))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, hasMaxImpulse) == 156, "Wrong offset for PxTGSSolverContactDesc.hasMaxImpulse, expected 156 got %v", offset_of(PxTGSSolverContactDesc, hasMaxImpulse))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, disableStrongFriction) == 157, "Wrong offset for PxTGSSolverContactDesc.disableStrongFriction, expected 157 got %v", offset_of(PxTGSSolverContactDesc, disableStrongFriction))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, hasForceThresholds) == 158, "Wrong offset for PxTGSSolverContactDesc.hasForceThresholds, expected 158 got %v", offset_of(PxTGSSolverContactDesc, hasForceThresholds))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, restDistance) == 160, "Wrong offset for PxTGSSolverContactDesc.restDistance, expected 160 got %v", offset_of(PxTGSSolverContactDesc, restDistance))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, maxCCDSeparation) == 164, "Wrong offset for PxTGSSolverContactDesc.maxCCDSeparation, expected 164 got %v", offset_of(PxTGSSolverContactDesc, maxCCDSeparation))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, frictionPtr) == 168, "Wrong offset for PxTGSSolverContactDesc.frictionPtr, expected 168 got %v", offset_of(PxTGSSolverContactDesc, frictionPtr))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, frictionCount) == 176, "Wrong offset for PxTGSSolverContactDesc.frictionCount, expected 176 got %v", offset_of(PxTGSSolverContactDesc, frictionCount))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, contactForces) == 184, "Wrong offset for PxTGSSolverContactDesc.contactForces, expected 184 got %v", offset_of(PxTGSSolverContactDesc, contactForces))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, startFrictionPatchIndex) == 192, "Wrong offset for PxTGSSolverContactDesc.startFrictionPatchIndex, expected 192 got %v", offset_of(PxTGSSolverContactDesc, startFrictionPatchIndex))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, numFrictionPatches) == 196, "Wrong offset for PxTGSSolverContactDesc.numFrictionPatches, expected 196 got %v", offset_of(PxTGSSolverContactDesc, numFrictionPatches))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, startContactPatchIndex) == 200, "Wrong offset for PxTGSSolverContactDesc.startContactPatchIndex, expected 200 got %v", offset_of(PxTGSSolverContactDesc, startContactPatchIndex))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, numContactPatches) == 204, "Wrong offset for PxTGSSolverContactDesc.numContactPatches, expected 204 got %v", offset_of(PxTGSSolverContactDesc, numContactPatches))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, axisConstraintCount) == 206, "Wrong offset for PxTGSSolverContactDesc.axisConstraintCount, expected 206 got %v", offset_of(PxTGSSolverContactDesc, axisConstraintCount))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, maxImpulse) == 208, "Wrong offset for PxTGSSolverContactDesc.maxImpulse, expected 208 got %v", offset_of(PxTGSSolverContactDesc, maxImpulse))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, torsionalPatchRadius) == 212, "Wrong offset for PxTGSSolverContactDesc.torsionalPatchRadius, expected 212 got %v", offset_of(PxTGSSolverContactDesc, torsionalPatchRadius))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, minTorsionalPatchRadius) == 216, "Wrong offset for PxTGSSolverContactDesc.minTorsionalPatchRadius, expected 216 got %v", offset_of(PxTGSSolverContactDesc, minTorsionalPatchRadius))
    testing.expectf(t, offset_of(PxTGSSolverContactDesc, offsetSlop) == 220, "Wrong offset for PxTGSSolverContactDesc.offsetSlop, expected 220 got %v", offset_of(PxTGSSolverContactDesc, offsetSlop))
    testing.expectf(t, size_of(PxTGSSolverContactDesc) == 224, "Wrong size for type PxTGSSolverContactDesc, expected 224 got %v", size_of(PxTGSSolverContactDesc))
}

@(test)
test_layout_PxArticulationTendonLimit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationTendonLimit, highLimit) == 4, "Wrong offset for PxArticulationTendonLimit.highLimit, expected 4 got %v", offset_of(PxArticulationTendonLimit, highLimit))
    testing.expectf(t, size_of(PxArticulationTendonLimit) == 8, "Wrong size for type PxArticulationTendonLimit, expected 8 got %v", size_of(PxArticulationTendonLimit))
}

@(test)
test_layout_PxArticulationAttachment :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationAttachment, userData) == 16, "Wrong offset for PxArticulationAttachment.userData, expected 16 got %v", offset_of(PxArticulationAttachment, userData))
    testing.expectf(t, size_of(PxArticulationAttachment) == 24, "Wrong size for type PxArticulationAttachment, expected 24 got %v", size_of(PxArticulationAttachment))
}

@(test)
test_layout_PxArticulationTendonJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationTendonJoint, userData) == 16, "Wrong offset for PxArticulationTendonJoint.userData, expected 16 got %v", offset_of(PxArticulationTendonJoint, userData))
    testing.expectf(t, size_of(PxArticulationTendonJoint) == 24, "Wrong size for type PxArticulationTendonJoint, expected 24 got %v", size_of(PxArticulationTendonJoint))
}

@(test)
test_layout_PxArticulationTendon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationTendon, userData) == 16, "Wrong offset for PxArticulationTendon.userData, expected 16 got %v", offset_of(PxArticulationTendon, userData))
    testing.expectf(t, size_of(PxArticulationTendon) == 24, "Wrong size for type PxArticulationTendon, expected 24 got %v", size_of(PxArticulationTendon))
}

@(test)
test_layout_PxArticulationSpatialTendon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxArticulationSpatialTendon) == 24, "Wrong size for type PxArticulationSpatialTendon, expected 24 got %v", size_of(PxArticulationSpatialTendon))
}

@(test)
test_layout_PxArticulationFixedTendon :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxArticulationFixedTendon) == 24, "Wrong size for type PxArticulationFixedTendon, expected 24 got %v", size_of(PxArticulationFixedTendon))
}

@(test)
test_layout_PxSpatialForce :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSpatialForce, pad0) == 12, "Wrong offset for PxSpatialForce.pad0, expected 12 got %v", offset_of(PxSpatialForce, pad0))
    testing.expectf(t, offset_of(PxSpatialForce, torque) == 16, "Wrong offset for PxSpatialForce.torque, expected 16 got %v", offset_of(PxSpatialForce, torque))
    testing.expectf(t, offset_of(PxSpatialForce, pad1) == 28, "Wrong offset for PxSpatialForce.pad1, expected 28 got %v", offset_of(PxSpatialForce, pad1))
    testing.expectf(t, size_of(PxSpatialForce) == 32, "Wrong size for type PxSpatialForce, expected 32 got %v", size_of(PxSpatialForce))
}

@(test)
test_layout_PxSpatialVelocity :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSpatialVelocity, pad0) == 12, "Wrong offset for PxSpatialVelocity.pad0, expected 12 got %v", offset_of(PxSpatialVelocity, pad0))
    testing.expectf(t, offset_of(PxSpatialVelocity, angular) == 16, "Wrong offset for PxSpatialVelocity.angular, expected 16 got %v", offset_of(PxSpatialVelocity, angular))
    testing.expectf(t, offset_of(PxSpatialVelocity, pad1) == 28, "Wrong offset for PxSpatialVelocity.pad1, expected 28 got %v", offset_of(PxSpatialVelocity, pad1))
    testing.expectf(t, size_of(PxSpatialVelocity) == 32, "Wrong size for type PxSpatialVelocity, expected 32 got %v", size_of(PxSpatialVelocity))
}

@(test)
test_layout_PxArticulationRootLinkData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationRootLinkData, worldLinVel) == 28, "Wrong offset for PxArticulationRootLinkData.worldLinVel, expected 28 got %v", offset_of(PxArticulationRootLinkData, worldLinVel))
    testing.expectf(t, offset_of(PxArticulationRootLinkData, worldAngVel) == 40, "Wrong offset for PxArticulationRootLinkData.worldAngVel, expected 40 got %v", offset_of(PxArticulationRootLinkData, worldAngVel))
    testing.expectf(t, offset_of(PxArticulationRootLinkData, worldLinAccel) == 52, "Wrong offset for PxArticulationRootLinkData.worldLinAccel, expected 52 got %v", offset_of(PxArticulationRootLinkData, worldLinAccel))
    testing.expectf(t, offset_of(PxArticulationRootLinkData, worldAngAccel) == 64, "Wrong offset for PxArticulationRootLinkData.worldAngAccel, expected 64 got %v", offset_of(PxArticulationRootLinkData, worldAngAccel))
    testing.expectf(t, size_of(PxArticulationRootLinkData) == 76, "Wrong size for type PxArticulationRootLinkData, expected 76 got %v", size_of(PxArticulationRootLinkData))
}

@(test)
test_layout_PxArticulationCache :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationCache, denseJacobian) == 8, "Wrong offset for PxArticulationCache.denseJacobian, expected 8 got %v", offset_of(PxArticulationCache, denseJacobian))
    testing.expectf(t, offset_of(PxArticulationCache, massMatrix) == 16, "Wrong offset for PxArticulationCache.massMatrix, expected 16 got %v", offset_of(PxArticulationCache, massMatrix))
    testing.expectf(t, offset_of(PxArticulationCache, jointVelocity) == 24, "Wrong offset for PxArticulationCache.jointVelocity, expected 24 got %v", offset_of(PxArticulationCache, jointVelocity))
    testing.expectf(t, offset_of(PxArticulationCache, jointAcceleration) == 32, "Wrong offset for PxArticulationCache.jointAcceleration, expected 32 got %v", offset_of(PxArticulationCache, jointAcceleration))
    testing.expectf(t, offset_of(PxArticulationCache, jointPosition) == 40, "Wrong offset for PxArticulationCache.jointPosition, expected 40 got %v", offset_of(PxArticulationCache, jointPosition))
    testing.expectf(t, offset_of(PxArticulationCache, jointForce) == 48, "Wrong offset for PxArticulationCache.jointForce, expected 48 got %v", offset_of(PxArticulationCache, jointForce))
    testing.expectf(t, offset_of(PxArticulationCache, jointSolverForces) == 56, "Wrong offset for PxArticulationCache.jointSolverForces, expected 56 got %v", offset_of(PxArticulationCache, jointSolverForces))
    testing.expectf(t, offset_of(PxArticulationCache, linkVelocity) == 64, "Wrong offset for PxArticulationCache.linkVelocity, expected 64 got %v", offset_of(PxArticulationCache, linkVelocity))
    testing.expectf(t, offset_of(PxArticulationCache, linkAcceleration) == 72, "Wrong offset for PxArticulationCache.linkAcceleration, expected 72 got %v", offset_of(PxArticulationCache, linkAcceleration))
    testing.expectf(t, offset_of(PxArticulationCache, rootLinkData) == 80, "Wrong offset for PxArticulationCache.rootLinkData, expected 80 got %v", offset_of(PxArticulationCache, rootLinkData))
    testing.expectf(t, offset_of(PxArticulationCache, sensorForces) == 88, "Wrong offset for PxArticulationCache.sensorForces, expected 88 got %v", offset_of(PxArticulationCache, sensorForces))
    testing.expectf(t, offset_of(PxArticulationCache, coefficientMatrix) == 96, "Wrong offset for PxArticulationCache.coefficientMatrix, expected 96 got %v", offset_of(PxArticulationCache, coefficientMatrix))
    testing.expectf(t, offset_of(PxArticulationCache, lambda) == 104, "Wrong offset for PxArticulationCache.lambda, expected 104 got %v", offset_of(PxArticulationCache, lambda))
    testing.expectf(t, offset_of(PxArticulationCache, scratchMemory) == 112, "Wrong offset for PxArticulationCache.scratchMemory, expected 112 got %v", offset_of(PxArticulationCache, scratchMemory))
    testing.expectf(t, offset_of(PxArticulationCache, scratchAllocator) == 120, "Wrong offset for PxArticulationCache.scratchAllocator, expected 120 got %v", offset_of(PxArticulationCache, scratchAllocator))
    testing.expectf(t, offset_of(PxArticulationCache, version) == 128, "Wrong offset for PxArticulationCache.version, expected 128 got %v", offset_of(PxArticulationCache, version))
    testing.expectf(t, size_of(PxArticulationCache) == 136, "Wrong size for type PxArticulationCache, expected 136 got %v", size_of(PxArticulationCache))
}

@(test)
test_layout_PxArticulationSensor :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationSensor, userData) == 16, "Wrong offset for PxArticulationSensor.userData, expected 16 got %v", offset_of(PxArticulationSensor, userData))
    testing.expectf(t, size_of(PxArticulationSensor) == 24, "Wrong size for type PxArticulationSensor, expected 24 got %v", size_of(PxArticulationSensor))
}

@(test)
test_layout_PxArticulationReducedCoordinate :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationReducedCoordinate, userData) == 16, "Wrong offset for PxArticulationReducedCoordinate.userData, expected 16 got %v", offset_of(PxArticulationReducedCoordinate, userData))
    testing.expectf(t, size_of(PxArticulationReducedCoordinate) == 24, "Wrong size for type PxArticulationReducedCoordinate, expected 24 got %v", size_of(PxArticulationReducedCoordinate))
}

@(test)
test_layout_PxArticulationJointReducedCoordinate :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxArticulationJointReducedCoordinate, userData) == 16, "Wrong offset for PxArticulationJointReducedCoordinate.userData, expected 16 got %v", offset_of(PxArticulationJointReducedCoordinate, userData))
    testing.expectf(t, size_of(PxArticulationJointReducedCoordinate) == 24, "Wrong size for type PxArticulationJointReducedCoordinate, expected 24 got %v", size_of(PxArticulationJointReducedCoordinate))
}

@(test)
test_layout_PxShape :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxShape, userData) == 16, "Wrong offset for PxShape.userData, expected 16 got %v", offset_of(PxShape, userData))
    testing.expectf(t, size_of(PxShape) == 24, "Wrong size for type PxShape, expected 24 got %v", size_of(PxShape))
}

@(test)
test_layout_PxRigidActor :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRigidActor) == 24, "Wrong size for type PxRigidActor, expected 24 got %v", size_of(PxRigidActor))
}

@(test)
test_layout_PxNodeIndex :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxNodeIndex) == 8, "Wrong size for type PxNodeIndex, expected 8 got %v", size_of(PxNodeIndex))
}

@(test)
test_layout_PxRigidBody :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRigidBody) == 24, "Wrong size for type PxRigidBody, expected 24 got %v", size_of(PxRigidBody))
}

@(test)
test_layout_PxArticulationLink :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxArticulationLink) == 24, "Wrong size for type PxArticulationLink, expected 24 got %v", size_of(PxArticulationLink))
}

@(test)
test_layout_PxConeLimitedConstraint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConeLimitedConstraint, mAngle) == 12, "Wrong offset for PxConeLimitedConstraint.mAngle, expected 12 got %v", offset_of(PxConeLimitedConstraint, mAngle))
    testing.expectf(t, offset_of(PxConeLimitedConstraint, mLowLimit) == 16, "Wrong offset for PxConeLimitedConstraint.mLowLimit, expected 16 got %v", offset_of(PxConeLimitedConstraint, mLowLimit))
    testing.expectf(t, offset_of(PxConeLimitedConstraint, mHighLimit) == 20, "Wrong offset for PxConeLimitedConstraint.mHighLimit, expected 20 got %v", offset_of(PxConeLimitedConstraint, mHighLimit))
    testing.expectf(t, size_of(PxConeLimitedConstraint) == 24, "Wrong size for type PxConeLimitedConstraint, expected 24 got %v", size_of(PxConeLimitedConstraint))
}

@(test)
test_layout_PxConeLimitParams :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConeLimitParams, axisAngle) == 16, "Wrong offset for PxConeLimitParams.axisAngle, expected 16 got %v", offset_of(PxConeLimitParams, axisAngle))
    testing.expectf(t, size_of(PxConeLimitParams) == 32, "Wrong size for type PxConeLimitParams, expected 32 got %v", size_of(PxConeLimitParams))
}

@(test)
test_layout_PxConstraintShaderTable :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConstraintShaderTable, visualize) == 16, "Wrong offset for PxConstraintShaderTable.visualize, expected 16 got %v", offset_of(PxConstraintShaderTable, visualize))
    testing.expectf(t, offset_of(PxConstraintShaderTable, flag) == 24, "Wrong offset for PxConstraintShaderTable.flag, expected 24 got %v", offset_of(PxConstraintShaderTable, flag))
    testing.expectf(t, size_of(PxConstraintShaderTable) == 32, "Wrong size for type PxConstraintShaderTable, expected 32 got %v", size_of(PxConstraintShaderTable))
}

@(test)
test_layout_PxConstraint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConstraint, userData) == 16, "Wrong offset for PxConstraint.userData, expected 16 got %v", offset_of(PxConstraint, userData))
    testing.expectf(t, size_of(PxConstraint) == 24, "Wrong size for type PxConstraint, expected 24 got %v", size_of(PxConstraint))
}

@(test)
test_layout_PxMassModificationProps :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxMassModificationProps, mInvInertiaScale0) == 4, "Wrong offset for PxMassModificationProps.mInvInertiaScale0, expected 4 got %v", offset_of(PxMassModificationProps, mInvInertiaScale0))
    testing.expectf(t, offset_of(PxMassModificationProps, mInvMassScale1) == 8, "Wrong offset for PxMassModificationProps.mInvMassScale1, expected 8 got %v", offset_of(PxMassModificationProps, mInvMassScale1))
    testing.expectf(t, offset_of(PxMassModificationProps, mInvInertiaScale1) == 12, "Wrong offset for PxMassModificationProps.mInvInertiaScale1, expected 12 got %v", offset_of(PxMassModificationProps, mInvInertiaScale1))
    testing.expectf(t, size_of(PxMassModificationProps) == 16, "Wrong size for type PxMassModificationProps, expected 16 got %v", size_of(PxMassModificationProps))
}

@(test)
test_layout_PxContactPatch :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPatch, normal) == 16, "Wrong offset for PxContactPatch.normal, expected 16 got %v", offset_of(PxContactPatch, normal))
    testing.expectf(t, offset_of(PxContactPatch, restitution) == 28, "Wrong offset for PxContactPatch.restitution, expected 28 got %v", offset_of(PxContactPatch, restitution))
    testing.expectf(t, offset_of(PxContactPatch, dynamicFriction) == 32, "Wrong offset for PxContactPatch.dynamicFriction, expected 32 got %v", offset_of(PxContactPatch, dynamicFriction))
    testing.expectf(t, offset_of(PxContactPatch, staticFriction) == 36, "Wrong offset for PxContactPatch.staticFriction, expected 36 got %v", offset_of(PxContactPatch, staticFriction))
    testing.expectf(t, offset_of(PxContactPatch, damping) == 40, "Wrong offset for PxContactPatch.damping, expected 40 got %v", offset_of(PxContactPatch, damping))
    testing.expectf(t, offset_of(PxContactPatch, startContactIndex) == 44, "Wrong offset for PxContactPatch.startContactIndex, expected 44 got %v", offset_of(PxContactPatch, startContactIndex))
    testing.expectf(t, offset_of(PxContactPatch, nbContacts) == 46, "Wrong offset for PxContactPatch.nbContacts, expected 46 got %v", offset_of(PxContactPatch, nbContacts))
    testing.expectf(t, offset_of(PxContactPatch, materialFlags) == 47, "Wrong offset for PxContactPatch.materialFlags, expected 47 got %v", offset_of(PxContactPatch, materialFlags))
    testing.expectf(t, offset_of(PxContactPatch, internalFlags) == 48, "Wrong offset for PxContactPatch.internalFlags, expected 48 got %v", offset_of(PxContactPatch, internalFlags))
    testing.expectf(t, offset_of(PxContactPatch, materialIndex0) == 50, "Wrong offset for PxContactPatch.materialIndex0, expected 50 got %v", offset_of(PxContactPatch, materialIndex0))
    testing.expectf(t, offset_of(PxContactPatch, materialIndex1) == 52, "Wrong offset for PxContactPatch.materialIndex1, expected 52 got %v", offset_of(PxContactPatch, materialIndex1))
    testing.expectf(t, size_of(PxContactPatch) == 64, "Wrong size for type PxContactPatch, expected 64 got %v", size_of(PxContactPatch))
}

@(test)
test_layout_PxContact :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContact, separation) == 12, "Wrong offset for PxContact.separation, expected 12 got %v", offset_of(PxContact, separation))
    testing.expectf(t, size_of(PxContact) == 16, "Wrong size for type PxContact, expected 16 got %v", size_of(PxContact))
}

@(test)
test_layout_PxExtendedContact :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxExtendedContact, targetVelocity) == 16, "Wrong offset for PxExtendedContact.targetVelocity, expected 16 got %v", offset_of(PxExtendedContact, targetVelocity))
    testing.expectf(t, offset_of(PxExtendedContact, maxImpulse) == 28, "Wrong offset for PxExtendedContact.maxImpulse, expected 28 got %v", offset_of(PxExtendedContact, maxImpulse))
    testing.expectf(t, size_of(PxExtendedContact) == 32, "Wrong size for type PxExtendedContact, expected 32 got %v", size_of(PxExtendedContact))
}

@(test)
test_layout_PxModifiableContact :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxModifiableContact, normal) == 32, "Wrong offset for PxModifiableContact.normal, expected 32 got %v", offset_of(PxModifiableContact, normal))
    testing.expectf(t, offset_of(PxModifiableContact, restitution) == 44, "Wrong offset for PxModifiableContact.restitution, expected 44 got %v", offset_of(PxModifiableContact, restitution))
    testing.expectf(t, offset_of(PxModifiableContact, materialFlags) == 48, "Wrong offset for PxModifiableContact.materialFlags, expected 48 got %v", offset_of(PxModifiableContact, materialFlags))
    testing.expectf(t, offset_of(PxModifiableContact, materialIndex0) == 52, "Wrong offset for PxModifiableContact.materialIndex0, expected 52 got %v", offset_of(PxModifiableContact, materialIndex0))
    testing.expectf(t, offset_of(PxModifiableContact, materialIndex1) == 54, "Wrong offset for PxModifiableContact.materialIndex1, expected 54 got %v", offset_of(PxModifiableContact, materialIndex1))
    testing.expectf(t, offset_of(PxModifiableContact, staticFriction) == 56, "Wrong offset for PxModifiableContact.staticFriction, expected 56 got %v", offset_of(PxModifiableContact, staticFriction))
    testing.expectf(t, offset_of(PxModifiableContact, dynamicFriction) == 60, "Wrong offset for PxModifiableContact.dynamicFriction, expected 60 got %v", offset_of(PxModifiableContact, dynamicFriction))
    testing.expectf(t, size_of(PxModifiableContact) == 64, "Wrong size for type PxModifiableContact, expected 64 got %v", size_of(PxModifiableContact))
}

@(test)
test_layout_PxContactStreamIterator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactStreamIterator, patch) == 16, "Wrong offset for PxContactStreamIterator.patch, expected 16 got %v", offset_of(PxContactStreamIterator, patch))
    testing.expectf(t, offset_of(PxContactStreamIterator, contact) == 24, "Wrong offset for PxContactStreamIterator.contact, expected 24 got %v", offset_of(PxContactStreamIterator, contact))
    testing.expectf(t, offset_of(PxContactStreamIterator, faceIndice) == 32, "Wrong offset for PxContactStreamIterator.faceIndice, expected 32 got %v", offset_of(PxContactStreamIterator, faceIndice))
    testing.expectf(t, offset_of(PxContactStreamIterator, totalPatches) == 40, "Wrong offset for PxContactStreamIterator.totalPatches, expected 40 got %v", offset_of(PxContactStreamIterator, totalPatches))
    testing.expectf(t, offset_of(PxContactStreamIterator, totalContacts) == 44, "Wrong offset for PxContactStreamIterator.totalContacts, expected 44 got %v", offset_of(PxContactStreamIterator, totalContacts))
    testing.expectf(t, offset_of(PxContactStreamIterator, nextContactIndex) == 48, "Wrong offset for PxContactStreamIterator.nextContactIndex, expected 48 got %v", offset_of(PxContactStreamIterator, nextContactIndex))
    testing.expectf(t, offset_of(PxContactStreamIterator, nextPatchIndex) == 52, "Wrong offset for PxContactStreamIterator.nextPatchIndex, expected 52 got %v", offset_of(PxContactStreamIterator, nextPatchIndex))
    testing.expectf(t, offset_of(PxContactStreamIterator, contactPatchHeaderSize) == 56, "Wrong offset for PxContactStreamIterator.contactPatchHeaderSize, expected 56 got %v", offset_of(PxContactStreamIterator, contactPatchHeaderSize))
    testing.expectf(t, offset_of(PxContactStreamIterator, contactPointSize) == 60, "Wrong offset for PxContactStreamIterator.contactPointSize, expected 60 got %v", offset_of(PxContactStreamIterator, contactPointSize))
    testing.expectf(t, offset_of(PxContactStreamIterator, mStreamFormat) == 64, "Wrong offset for PxContactStreamIterator.mStreamFormat, expected 64 got %v", offset_of(PxContactStreamIterator, mStreamFormat))
    testing.expectf(t, offset_of(PxContactStreamIterator, forceNoResponse) == 68, "Wrong offset for PxContactStreamIterator.forceNoResponse, expected 68 got %v", offset_of(PxContactStreamIterator, forceNoResponse))
    testing.expectf(t, offset_of(PxContactStreamIterator, pointStepped) == 72, "Wrong offset for PxContactStreamIterator.pointStepped, expected 72 got %v", offset_of(PxContactStreamIterator, pointStepped))
    testing.expectf(t, offset_of(PxContactStreamIterator, hasFaceIndices) == 76, "Wrong offset for PxContactStreamIterator.hasFaceIndices, expected 76 got %v", offset_of(PxContactStreamIterator, hasFaceIndices))
    testing.expectf(t, size_of(PxContactStreamIterator) == 80, "Wrong size for type PxContactStreamIterator, expected 80 got %v", size_of(PxContactStreamIterator))
}

@(test)
test_layout_PxGpuContactPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGpuContactPair, contactPoints) == 8, "Wrong offset for PxGpuContactPair.contactPoints, expected 8 got %v", offset_of(PxGpuContactPair, contactPoints))
    testing.expectf(t, offset_of(PxGpuContactPair, contactForces) == 16, "Wrong offset for PxGpuContactPair.contactForces, expected 16 got %v", offset_of(PxGpuContactPair, contactForces))
    testing.expectf(t, offset_of(PxGpuContactPair, transformCacheRef0) == 24, "Wrong offset for PxGpuContactPair.transformCacheRef0, expected 24 got %v", offset_of(PxGpuContactPair, transformCacheRef0))
    testing.expectf(t, offset_of(PxGpuContactPair, transformCacheRef1) == 28, "Wrong offset for PxGpuContactPair.transformCacheRef1, expected 28 got %v", offset_of(PxGpuContactPair, transformCacheRef1))
    testing.expectf(t, offset_of(PxGpuContactPair, nodeIndex0) == 32, "Wrong offset for PxGpuContactPair.nodeIndex0, expected 32 got %v", offset_of(PxGpuContactPair, nodeIndex0))
    testing.expectf(t, offset_of(PxGpuContactPair, nodeIndex1) == 40, "Wrong offset for PxGpuContactPair.nodeIndex1, expected 40 got %v", offset_of(PxGpuContactPair, nodeIndex1))
    testing.expectf(t, offset_of(PxGpuContactPair, actor0) == 48, "Wrong offset for PxGpuContactPair.actor0, expected 48 got %v", offset_of(PxGpuContactPair, actor0))
    testing.expectf(t, offset_of(PxGpuContactPair, actor1) == 56, "Wrong offset for PxGpuContactPair.actor1, expected 56 got %v", offset_of(PxGpuContactPair, actor1))
    testing.expectf(t, offset_of(PxGpuContactPair, nbContacts) == 64, "Wrong offset for PxGpuContactPair.nbContacts, expected 64 got %v", offset_of(PxGpuContactPair, nbContacts))
    testing.expectf(t, offset_of(PxGpuContactPair, nbPatches) == 66, "Wrong offset for PxGpuContactPair.nbPatches, expected 66 got %v", offset_of(PxGpuContactPair, nbPatches))
    testing.expectf(t, size_of(PxGpuContactPair) == 72, "Wrong size for type PxGpuContactPair, expected 72 got %v", size_of(PxGpuContactPair))
}

@(test)
test_layout_PxContactSet :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxContactSet) == 16, "Wrong size for type PxContactSet, expected 16 got %v", size_of(PxContactSet))
}

@(test)
test_layout_PxContactModifyPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactModifyPair, contacts) == 88, "Wrong offset for PxContactModifyPair.contacts, expected 88 got %v", offset_of(PxContactModifyPair, contacts))
    testing.expectf(t, size_of(PxContactModifyPair) == 104, "Wrong size for type PxContactModifyPair, expected 104 got %v", size_of(PxContactModifyPair))
}

@(test)
test_layout_PxContactModifyCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxContactModifyCallback) == 8, "Wrong size for type PxContactModifyCallback, expected 8 got %v", size_of(PxContactModifyCallback))
}

@(test)
test_layout_PxCCDContactModifyCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCCDContactModifyCallback) == 8, "Wrong size for type PxCCDContactModifyCallback, expected 8 got %v", size_of(PxCCDContactModifyCallback))
}

@(test)
test_layout_PxDeletionListener :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDeletionListener) == 8, "Wrong size for type PxDeletionListener, expected 8 got %v", size_of(PxDeletionListener))
}

@(test)
test_layout_PxBaseMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBaseMaterial, userData) == 16, "Wrong offset for PxBaseMaterial.userData, expected 16 got %v", offset_of(PxBaseMaterial, userData))
    testing.expectf(t, size_of(PxBaseMaterial) == 24, "Wrong size for type PxBaseMaterial, expected 24 got %v", size_of(PxBaseMaterial))
}

@(test)
test_layout_PxFEMMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxFEMMaterial) == 24, "Wrong size for type PxFEMMaterial, expected 24 got %v", size_of(PxFEMMaterial))
}

@(test)
test_layout_PxFilterData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxFilterData, word1) == 4, "Wrong offset for PxFilterData.word1, expected 4 got %v", offset_of(PxFilterData, word1))
    testing.expectf(t, offset_of(PxFilterData, word2) == 8, "Wrong offset for PxFilterData.word2, expected 8 got %v", offset_of(PxFilterData, word2))
    testing.expectf(t, offset_of(PxFilterData, word3) == 12, "Wrong offset for PxFilterData.word3, expected 12 got %v", offset_of(PxFilterData, word3))
    testing.expectf(t, size_of(PxFilterData) == 16, "Wrong size for type PxFilterData, expected 16 got %v", size_of(PxFilterData))
}

@(test)
test_layout_PxSimulationFilterCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSimulationFilterCallback) == 8, "Wrong size for type PxSimulationFilterCallback, expected 8 got %v", size_of(PxSimulationFilterCallback))
}

@(test)
test_layout_PxParticleRigidFilterPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxParticleRigidFilterPair, mID1) == 8, "Wrong offset for PxParticleRigidFilterPair.mID1, expected 8 got %v", offset_of(PxParticleRigidFilterPair, mID1))
    testing.expectf(t, size_of(PxParticleRigidFilterPair) == 16, "Wrong size for type PxParticleRigidFilterPair, expected 16 got %v", size_of(PxParticleRigidFilterPair))
}

@(test)
test_layout_PxLockedData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxLockedData) == 8, "Wrong size for type PxLockedData, expected 8 got %v", size_of(PxLockedData))
}

@(test)
test_layout_PxMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxMaterial) == 24, "Wrong size for type PxMaterial, expected 24 got %v", size_of(PxMaterial))
}

@(test)
test_layout_PxGpuParticleBufferIndexPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGpuParticleBufferIndexPair, bufferIndex) == 4, "Wrong offset for PxGpuParticleBufferIndexPair.bufferIndex, expected 4 got %v", offset_of(PxGpuParticleBufferIndexPair, bufferIndex))
    testing.expectf(t, size_of(PxGpuParticleBufferIndexPair) == 8, "Wrong size for type PxGpuParticleBufferIndexPair, expected 8 got %v", size_of(PxGpuParticleBufferIndexPair))
}

@(test)
test_layout_PxParticleVolume :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxParticleVolume, particleIndicesOffset) == 24, "Wrong offset for PxParticleVolume.particleIndicesOffset, expected 24 got %v", offset_of(PxParticleVolume, particleIndicesOffset))
    testing.expectf(t, offset_of(PxParticleVolume, numParticles) == 28, "Wrong offset for PxParticleVolume.numParticles, expected 28 got %v", offset_of(PxParticleVolume, numParticles))
    testing.expectf(t, size_of(PxParticleVolume) == 32, "Wrong size for type PxParticleVolume, expected 32 got %v", size_of(PxParticleVolume))
}

@(test)
test_layout_PxDiffuseParticleParams :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDiffuseParticleParams, lifetime) == 4, "Wrong offset for PxDiffuseParticleParams.lifetime, expected 4 got %v", offset_of(PxDiffuseParticleParams, lifetime))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, airDrag) == 8, "Wrong offset for PxDiffuseParticleParams.airDrag, expected 8 got %v", offset_of(PxDiffuseParticleParams, airDrag))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, bubbleDrag) == 12, "Wrong offset for PxDiffuseParticleParams.bubbleDrag, expected 12 got %v", offset_of(PxDiffuseParticleParams, bubbleDrag))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, buoyancy) == 16, "Wrong offset for PxDiffuseParticleParams.buoyancy, expected 16 got %v", offset_of(PxDiffuseParticleParams, buoyancy))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, kineticEnergyWeight) == 20, "Wrong offset for PxDiffuseParticleParams.kineticEnergyWeight, expected 20 got %v", offset_of(PxDiffuseParticleParams, kineticEnergyWeight))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, pressureWeight) == 24, "Wrong offset for PxDiffuseParticleParams.pressureWeight, expected 24 got %v", offset_of(PxDiffuseParticleParams, pressureWeight))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, divergenceWeight) == 28, "Wrong offset for PxDiffuseParticleParams.divergenceWeight, expected 28 got %v", offset_of(PxDiffuseParticleParams, divergenceWeight))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, collisionDecay) == 32, "Wrong offset for PxDiffuseParticleParams.collisionDecay, expected 32 got %v", offset_of(PxDiffuseParticleParams, collisionDecay))
    testing.expectf(t, offset_of(PxDiffuseParticleParams, useAccurateVelocity) == 36, "Wrong offset for PxDiffuseParticleParams.useAccurateVelocity, expected 36 got %v", offset_of(PxDiffuseParticleParams, useAccurateVelocity))
    testing.expectf(t, size_of(PxDiffuseParticleParams) == 40, "Wrong size for type PxDiffuseParticleParams, expected 40 got %v", size_of(PxDiffuseParticleParams))
}

@(test)
test_layout_PxParticleSpring :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxParticleSpring, ind1) == 4, "Wrong offset for PxParticleSpring.ind1, expected 4 got %v", offset_of(PxParticleSpring, ind1))
    testing.expectf(t, offset_of(PxParticleSpring, length) == 8, "Wrong offset for PxParticleSpring.length, expected 8 got %v", offset_of(PxParticleSpring, length))
    testing.expectf(t, offset_of(PxParticleSpring, stiffness) == 12, "Wrong offset for PxParticleSpring.stiffness, expected 12 got %v", offset_of(PxParticleSpring, stiffness))
    testing.expectf(t, offset_of(PxParticleSpring, damping) == 16, "Wrong offset for PxParticleSpring.damping, expected 16 got %v", offset_of(PxParticleSpring, damping))
    testing.expectf(t, offset_of(PxParticleSpring, pad) == 20, "Wrong offset for PxParticleSpring.pad, expected 20 got %v", offset_of(PxParticleSpring, pad))
    testing.expectf(t, size_of(PxParticleSpring) == 24, "Wrong size for type PxParticleSpring, expected 24 got %v", size_of(PxParticleSpring))
}

@(test)
test_layout_PxParticleMaterial :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxParticleMaterial) == 24, "Wrong size for type PxParticleMaterial, expected 24 got %v", size_of(PxParticleMaterial))
}

@(test)
test_layout_PxPhysics :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPhysics) == 8, "Wrong size for type PxPhysics, expected 8 got %v", size_of(PxPhysics))
}

@(test)
test_layout_PxActorShape :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxActorShape, shape) == 8, "Wrong offset for PxActorShape.shape, expected 8 got %v", offset_of(PxActorShape, shape))
    testing.expectf(t, size_of(PxActorShape) == 16, "Wrong size for type PxActorShape, expected 16 got %v", size_of(PxActorShape))
}

@(test)
test_layout_PxRaycastHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRaycastHit) == 64, "Wrong size for type PxRaycastHit, expected 64 got %v", size_of(PxRaycastHit))
}

@(test)
test_layout_PxOverlapHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxOverlapHit) == 24, "Wrong size for type PxOverlapHit, expected 24 got %v", size_of(PxOverlapHit))
}

@(test)
test_layout_PxSweepHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSweepHit) == 56, "Wrong size for type PxSweepHit, expected 56 got %v", size_of(PxSweepHit))
}

@(test)
test_layout_PxRaycastCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxRaycastCallback, block) == 8, "Wrong offset for PxRaycastCallback.block, expected 8 got %v", offset_of(PxRaycastCallback, block))
    testing.expectf(t, offset_of(PxRaycastCallback, hasBlock) == 72, "Wrong offset for PxRaycastCallback.hasBlock, expected 72 got %v", offset_of(PxRaycastCallback, hasBlock))
    testing.expectf(t, offset_of(PxRaycastCallback, touches) == 80, "Wrong offset for PxRaycastCallback.touches, expected 80 got %v", offset_of(PxRaycastCallback, touches))
    testing.expectf(t, offset_of(PxRaycastCallback, maxNbTouches) == 88, "Wrong offset for PxRaycastCallback.maxNbTouches, expected 88 got %v", offset_of(PxRaycastCallback, maxNbTouches))
    testing.expectf(t, offset_of(PxRaycastCallback, nbTouches) == 92, "Wrong offset for PxRaycastCallback.nbTouches, expected 92 got %v", offset_of(PxRaycastCallback, nbTouches))
    testing.expectf(t, size_of(PxRaycastCallback) == 96, "Wrong size for type PxRaycastCallback, expected 96 got %v", size_of(PxRaycastCallback))
}

@(test)
test_layout_PxOverlapCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxOverlapCallback, block) == 8, "Wrong offset for PxOverlapCallback.block, expected 8 got %v", offset_of(PxOverlapCallback, block))
    testing.expectf(t, offset_of(PxOverlapCallback, hasBlock) == 32, "Wrong offset for PxOverlapCallback.hasBlock, expected 32 got %v", offset_of(PxOverlapCallback, hasBlock))
    testing.expectf(t, offset_of(PxOverlapCallback, touches) == 40, "Wrong offset for PxOverlapCallback.touches, expected 40 got %v", offset_of(PxOverlapCallback, touches))
    testing.expectf(t, offset_of(PxOverlapCallback, maxNbTouches) == 48, "Wrong offset for PxOverlapCallback.maxNbTouches, expected 48 got %v", offset_of(PxOverlapCallback, maxNbTouches))
    testing.expectf(t, offset_of(PxOverlapCallback, nbTouches) == 52, "Wrong offset for PxOverlapCallback.nbTouches, expected 52 got %v", offset_of(PxOverlapCallback, nbTouches))
    testing.expectf(t, size_of(PxOverlapCallback) == 56, "Wrong size for type PxOverlapCallback, expected 56 got %v", size_of(PxOverlapCallback))
}

@(test)
test_layout_PxSweepCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSweepCallback, block) == 8, "Wrong offset for PxSweepCallback.block, expected 8 got %v", offset_of(PxSweepCallback, block))
    testing.expectf(t, offset_of(PxSweepCallback, hasBlock) == 64, "Wrong offset for PxSweepCallback.hasBlock, expected 64 got %v", offset_of(PxSweepCallback, hasBlock))
    testing.expectf(t, offset_of(PxSweepCallback, touches) == 72, "Wrong offset for PxSweepCallback.touches, expected 72 got %v", offset_of(PxSweepCallback, touches))
    testing.expectf(t, offset_of(PxSweepCallback, maxNbTouches) == 80, "Wrong offset for PxSweepCallback.maxNbTouches, expected 80 got %v", offset_of(PxSweepCallback, maxNbTouches))
    testing.expectf(t, offset_of(PxSweepCallback, nbTouches) == 84, "Wrong offset for PxSweepCallback.nbTouches, expected 84 got %v", offset_of(PxSweepCallback, nbTouches))
    testing.expectf(t, size_of(PxSweepCallback) == 88, "Wrong size for type PxSweepCallback, expected 88 got %v", size_of(PxSweepCallback))
}

@(test)
test_layout_PxRaycastBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxRaycastBuffer, block) == 8, "Wrong offset for PxRaycastBuffer.block, expected 8 got %v", offset_of(PxRaycastBuffer, block))
    testing.expectf(t, offset_of(PxRaycastBuffer, hasBlock) == 72, "Wrong offset for PxRaycastBuffer.hasBlock, expected 72 got %v", offset_of(PxRaycastBuffer, hasBlock))
    testing.expectf(t, offset_of(PxRaycastBuffer, touches) == 80, "Wrong offset for PxRaycastBuffer.touches, expected 80 got %v", offset_of(PxRaycastBuffer, touches))
    testing.expectf(t, offset_of(PxRaycastBuffer, maxNbTouches) == 88, "Wrong offset for PxRaycastBuffer.maxNbTouches, expected 88 got %v", offset_of(PxRaycastBuffer, maxNbTouches))
    testing.expectf(t, offset_of(PxRaycastBuffer, nbTouches) == 92, "Wrong offset for PxRaycastBuffer.nbTouches, expected 92 got %v", offset_of(PxRaycastBuffer, nbTouches))
    testing.expectf(t, size_of(PxRaycastBuffer) == 96, "Wrong size for type PxRaycastBuffer, expected 96 got %v", size_of(PxRaycastBuffer))
}

@(test)
test_layout_PxOverlapBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxOverlapBuffer, block) == 8, "Wrong offset for PxOverlapBuffer.block, expected 8 got %v", offset_of(PxOverlapBuffer, block))
    testing.expectf(t, offset_of(PxOverlapBuffer, hasBlock) == 32, "Wrong offset for PxOverlapBuffer.hasBlock, expected 32 got %v", offset_of(PxOverlapBuffer, hasBlock))
    testing.expectf(t, offset_of(PxOverlapBuffer, touches) == 40, "Wrong offset for PxOverlapBuffer.touches, expected 40 got %v", offset_of(PxOverlapBuffer, touches))
    testing.expectf(t, offset_of(PxOverlapBuffer, maxNbTouches) == 48, "Wrong offset for PxOverlapBuffer.maxNbTouches, expected 48 got %v", offset_of(PxOverlapBuffer, maxNbTouches))
    testing.expectf(t, offset_of(PxOverlapBuffer, nbTouches) == 52, "Wrong offset for PxOverlapBuffer.nbTouches, expected 52 got %v", offset_of(PxOverlapBuffer, nbTouches))
    testing.expectf(t, size_of(PxOverlapBuffer) == 56, "Wrong size for type PxOverlapBuffer, expected 56 got %v", size_of(PxOverlapBuffer))
}

@(test)
test_layout_PxSweepBuffer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSweepBuffer, block) == 8, "Wrong offset for PxSweepBuffer.block, expected 8 got %v", offset_of(PxSweepBuffer, block))
    testing.expectf(t, offset_of(PxSweepBuffer, hasBlock) == 64, "Wrong offset for PxSweepBuffer.hasBlock, expected 64 got %v", offset_of(PxSweepBuffer, hasBlock))
    testing.expectf(t, offset_of(PxSweepBuffer, touches) == 72, "Wrong offset for PxSweepBuffer.touches, expected 72 got %v", offset_of(PxSweepBuffer, touches))
    testing.expectf(t, offset_of(PxSweepBuffer, maxNbTouches) == 80, "Wrong offset for PxSweepBuffer.maxNbTouches, expected 80 got %v", offset_of(PxSweepBuffer, maxNbTouches))
    testing.expectf(t, offset_of(PxSweepBuffer, nbTouches) == 84, "Wrong offset for PxSweepBuffer.nbTouches, expected 84 got %v", offset_of(PxSweepBuffer, nbTouches))
    testing.expectf(t, size_of(PxSweepBuffer) == 88, "Wrong size for type PxSweepBuffer, expected 88 got %v", size_of(PxSweepBuffer))
}

@(test)
test_layout_PxQueryCache :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxQueryCache, actor) == 8, "Wrong offset for PxQueryCache.actor, expected 8 got %v", offset_of(PxQueryCache, actor))
    testing.expectf(t, offset_of(PxQueryCache, faceIndex) == 16, "Wrong offset for PxQueryCache.faceIndex, expected 16 got %v", offset_of(PxQueryCache, faceIndex))
    testing.expectf(t, size_of(PxQueryCache) == 24, "Wrong size for type PxQueryCache, expected 24 got %v", size_of(PxQueryCache))
}

@(test)
test_layout_PxQueryFilterData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxQueryFilterData, flags) == 16, "Wrong offset for PxQueryFilterData.flags, expected 16 got %v", offset_of(PxQueryFilterData, flags))
    testing.expectf(t, size_of(PxQueryFilterData) == 20, "Wrong size for type PxQueryFilterData, expected 20 got %v", size_of(PxQueryFilterData))
}

@(test)
test_layout_PxQueryFilterCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxQueryFilterCallback) == 8, "Wrong size for type PxQueryFilterCallback, expected 8 got %v", size_of(PxQueryFilterCallback))
}

@(test)
test_layout_PxRigidDynamic :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRigidDynamic) == 24, "Wrong size for type PxRigidDynamic, expected 24 got %v", size_of(PxRigidDynamic))
}

@(test)
test_layout_PxRigidStatic :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRigidStatic) == 24, "Wrong size for type PxRigidStatic, expected 24 got %v", size_of(PxRigidStatic))
}

@(test)
test_layout_PxSceneQueryDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSceneQueryDesc, dynamicStructure) == 4, "Wrong offset for PxSceneQueryDesc.dynamicStructure, expected 4 got %v", offset_of(PxSceneQueryDesc, dynamicStructure))
    testing.expectf(t, offset_of(PxSceneQueryDesc, dynamicTreeRebuildRateHint) == 8, "Wrong offset for PxSceneQueryDesc.dynamicTreeRebuildRateHint, expected 8 got %v", offset_of(PxSceneQueryDesc, dynamicTreeRebuildRateHint))
    testing.expectf(t, offset_of(PxSceneQueryDesc, dynamicTreeSecondaryPruner) == 12, "Wrong offset for PxSceneQueryDesc.dynamicTreeSecondaryPruner, expected 12 got %v", offset_of(PxSceneQueryDesc, dynamicTreeSecondaryPruner))
    testing.expectf(t, offset_of(PxSceneQueryDesc, staticBVHBuildStrategy) == 16, "Wrong offset for PxSceneQueryDesc.staticBVHBuildStrategy, expected 16 got %v", offset_of(PxSceneQueryDesc, staticBVHBuildStrategy))
    testing.expectf(t, offset_of(PxSceneQueryDesc, dynamicBVHBuildStrategy) == 20, "Wrong offset for PxSceneQueryDesc.dynamicBVHBuildStrategy, expected 20 got %v", offset_of(PxSceneQueryDesc, dynamicBVHBuildStrategy))
    testing.expectf(t, offset_of(PxSceneQueryDesc, staticNbObjectsPerNode) == 24, "Wrong offset for PxSceneQueryDesc.staticNbObjectsPerNode, expected 24 got %v", offset_of(PxSceneQueryDesc, staticNbObjectsPerNode))
    testing.expectf(t, offset_of(PxSceneQueryDesc, dynamicNbObjectsPerNode) == 28, "Wrong offset for PxSceneQueryDesc.dynamicNbObjectsPerNode, expected 28 got %v", offset_of(PxSceneQueryDesc, dynamicNbObjectsPerNode))
    testing.expectf(t, offset_of(PxSceneQueryDesc, sceneQueryUpdateMode) == 32, "Wrong offset for PxSceneQueryDesc.sceneQueryUpdateMode, expected 32 got %v", offset_of(PxSceneQueryDesc, sceneQueryUpdateMode))
    testing.expectf(t, size_of(PxSceneQueryDesc) == 36, "Wrong size for type PxSceneQueryDesc, expected 36 got %v", size_of(PxSceneQueryDesc))
}

@(test)
test_layout_PxSceneQuerySystemBase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSceneQuerySystemBase) == 8, "Wrong size for type PxSceneQuerySystemBase, expected 8 got %v", size_of(PxSceneQuerySystemBase))
}

@(test)
test_layout_PxSceneSQSystem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSceneSQSystem) == 8, "Wrong size for type PxSceneSQSystem, expected 8 got %v", size_of(PxSceneSQSystem))
}

@(test)
test_layout_PxSceneQuerySystem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSceneQuerySystem) == 8, "Wrong size for type PxSceneQuerySystem, expected 8 got %v", size_of(PxSceneQuerySystem))
}

@(test)
test_layout_PxBroadPhaseRegion :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBroadPhaseRegion, mUserData) == 24, "Wrong offset for PxBroadPhaseRegion.mUserData, expected 24 got %v", offset_of(PxBroadPhaseRegion, mUserData))
    testing.expectf(t, size_of(PxBroadPhaseRegion) == 32, "Wrong size for type PxBroadPhaseRegion, expected 32 got %v", size_of(PxBroadPhaseRegion))
}

@(test)
test_layout_PxBroadPhaseRegionInfo :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBroadPhaseRegionInfo, mNbStaticObjects) == 32, "Wrong offset for PxBroadPhaseRegionInfo.mNbStaticObjects, expected 32 got %v", offset_of(PxBroadPhaseRegionInfo, mNbStaticObjects))
    testing.expectf(t, offset_of(PxBroadPhaseRegionInfo, mNbDynamicObjects) == 36, "Wrong offset for PxBroadPhaseRegionInfo.mNbDynamicObjects, expected 36 got %v", offset_of(PxBroadPhaseRegionInfo, mNbDynamicObjects))
    testing.expectf(t, offset_of(PxBroadPhaseRegionInfo, mActive) == 40, "Wrong offset for PxBroadPhaseRegionInfo.mActive, expected 40 got %v", offset_of(PxBroadPhaseRegionInfo, mActive))
    testing.expectf(t, offset_of(PxBroadPhaseRegionInfo, mOverlap) == 41, "Wrong offset for PxBroadPhaseRegionInfo.mOverlap, expected 41 got %v", offset_of(PxBroadPhaseRegionInfo, mOverlap))
    testing.expectf(t, size_of(PxBroadPhaseRegionInfo) == 48, "Wrong size for type PxBroadPhaseRegionInfo, expected 48 got %v", size_of(PxBroadPhaseRegionInfo))
}

@(test)
test_layout_PxBroadPhaseCaps :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBroadPhaseCaps) == 4, "Wrong size for type PxBroadPhaseCaps, expected 4 got %v", size_of(PxBroadPhaseCaps))
}

@(test)
test_layout_PxBroadPhaseDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBroadPhaseDesc, mContextID) == 8, "Wrong offset for PxBroadPhaseDesc.mContextID, expected 8 got %v", offset_of(PxBroadPhaseDesc, mContextID))
    testing.expectf(t, offset_of(PxBroadPhaseDesc, mFoundLostPairsCapacity) == 24, "Wrong offset for PxBroadPhaseDesc.mFoundLostPairsCapacity, expected 24 got %v", offset_of(PxBroadPhaseDesc, mFoundLostPairsCapacity))
    testing.expectf(t, offset_of(PxBroadPhaseDesc, mDiscardStaticVsKinematic) == 28, "Wrong offset for PxBroadPhaseDesc.mDiscardStaticVsKinematic, expected 28 got %v", offset_of(PxBroadPhaseDesc, mDiscardStaticVsKinematic))
    testing.expectf(t, offset_of(PxBroadPhaseDesc, mDiscardKinematicVsKinematic) == 29, "Wrong offset for PxBroadPhaseDesc.mDiscardKinematicVsKinematic, expected 29 got %v", offset_of(PxBroadPhaseDesc, mDiscardKinematicVsKinematic))
    testing.expectf(t, size_of(PxBroadPhaseDesc) == 32, "Wrong size for type PxBroadPhaseDesc, expected 32 got %v", size_of(PxBroadPhaseDesc))
}

@(test)
test_layout_PxBroadPhaseUpdateData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mNbCreated) == 8, "Wrong offset for PxBroadPhaseUpdateData.mNbCreated, expected 8 got %v", offset_of(PxBroadPhaseUpdateData, mNbCreated))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mUpdated) == 16, "Wrong offset for PxBroadPhaseUpdateData.mUpdated, expected 16 got %v", offset_of(PxBroadPhaseUpdateData, mUpdated))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mNbUpdated) == 24, "Wrong offset for PxBroadPhaseUpdateData.mNbUpdated, expected 24 got %v", offset_of(PxBroadPhaseUpdateData, mNbUpdated))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mRemoved) == 32, "Wrong offset for PxBroadPhaseUpdateData.mRemoved, expected 32 got %v", offset_of(PxBroadPhaseUpdateData, mRemoved))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mNbRemoved) == 40, "Wrong offset for PxBroadPhaseUpdateData.mNbRemoved, expected 40 got %v", offset_of(PxBroadPhaseUpdateData, mNbRemoved))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mBounds) == 48, "Wrong offset for PxBroadPhaseUpdateData.mBounds, expected 48 got %v", offset_of(PxBroadPhaseUpdateData, mBounds))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mGroups) == 56, "Wrong offset for PxBroadPhaseUpdateData.mGroups, expected 56 got %v", offset_of(PxBroadPhaseUpdateData, mGroups))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mDistances) == 64, "Wrong offset for PxBroadPhaseUpdateData.mDistances, expected 64 got %v", offset_of(PxBroadPhaseUpdateData, mDistances))
    testing.expectf(t, offset_of(PxBroadPhaseUpdateData, mCapacity) == 72, "Wrong offset for PxBroadPhaseUpdateData.mCapacity, expected 72 got %v", offset_of(PxBroadPhaseUpdateData, mCapacity))
    testing.expectf(t, size_of(PxBroadPhaseUpdateData) == 80, "Wrong size for type PxBroadPhaseUpdateData, expected 80 got %v", size_of(PxBroadPhaseUpdateData))
}

@(test)
test_layout_PxBroadPhasePair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBroadPhasePair, mID1) == 4, "Wrong offset for PxBroadPhasePair.mID1, expected 4 got %v", offset_of(PxBroadPhasePair, mID1))
    testing.expectf(t, size_of(PxBroadPhasePair) == 8, "Wrong size for type PxBroadPhasePair, expected 8 got %v", size_of(PxBroadPhasePair))
}

@(test)
test_layout_PxBroadPhaseResults :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBroadPhaseResults, mCreatedPairs) == 8, "Wrong offset for PxBroadPhaseResults.mCreatedPairs, expected 8 got %v", offset_of(PxBroadPhaseResults, mCreatedPairs))
    testing.expectf(t, offset_of(PxBroadPhaseResults, mNbDeletedPairs) == 16, "Wrong offset for PxBroadPhaseResults.mNbDeletedPairs, expected 16 got %v", offset_of(PxBroadPhaseResults, mNbDeletedPairs))
    testing.expectf(t, offset_of(PxBroadPhaseResults, mDeletedPairs) == 24, "Wrong offset for PxBroadPhaseResults.mDeletedPairs, expected 24 got %v", offset_of(PxBroadPhaseResults, mDeletedPairs))
    testing.expectf(t, size_of(PxBroadPhaseResults) == 32, "Wrong size for type PxBroadPhaseResults, expected 32 got %v", size_of(PxBroadPhaseResults))
}

@(test)
test_layout_PxBroadPhaseRegions :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBroadPhaseRegions) == 8, "Wrong size for type PxBroadPhaseRegions, expected 8 got %v", size_of(PxBroadPhaseRegions))
}

@(test)
test_layout_PxBroadPhase :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBroadPhase) == 8, "Wrong size for type PxBroadPhase, expected 8 got %v", size_of(PxBroadPhase))
}

@(test)
test_layout_PxAABBManager :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxAABBManager) == 8, "Wrong size for type PxAABBManager, expected 8 got %v", size_of(PxAABBManager))
}

@(test)
test_layout_PxSceneLimits :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSceneLimits, maxNbBodies) == 4, "Wrong offset for PxSceneLimits.maxNbBodies, expected 4 got %v", offset_of(PxSceneLimits, maxNbBodies))
    testing.expectf(t, offset_of(PxSceneLimits, maxNbStaticShapes) == 8, "Wrong offset for PxSceneLimits.maxNbStaticShapes, expected 8 got %v", offset_of(PxSceneLimits, maxNbStaticShapes))
    testing.expectf(t, offset_of(PxSceneLimits, maxNbDynamicShapes) == 12, "Wrong offset for PxSceneLimits.maxNbDynamicShapes, expected 12 got %v", offset_of(PxSceneLimits, maxNbDynamicShapes))
    testing.expectf(t, offset_of(PxSceneLimits, maxNbAggregates) == 16, "Wrong offset for PxSceneLimits.maxNbAggregates, expected 16 got %v", offset_of(PxSceneLimits, maxNbAggregates))
    testing.expectf(t, offset_of(PxSceneLimits, maxNbConstraints) == 20, "Wrong offset for PxSceneLimits.maxNbConstraints, expected 20 got %v", offset_of(PxSceneLimits, maxNbConstraints))
    testing.expectf(t, offset_of(PxSceneLimits, maxNbRegions) == 24, "Wrong offset for PxSceneLimits.maxNbRegions, expected 24 got %v", offset_of(PxSceneLimits, maxNbRegions))
    testing.expectf(t, offset_of(PxSceneLimits, maxNbBroadPhaseOverlaps) == 28, "Wrong offset for PxSceneLimits.maxNbBroadPhaseOverlaps, expected 28 got %v", offset_of(PxSceneLimits, maxNbBroadPhaseOverlaps))
    testing.expectf(t, size_of(PxSceneLimits) == 32, "Wrong size for type PxSceneLimits, expected 32 got %v", size_of(PxSceneLimits))
}

@(test)
test_layout_PxgDynamicsMemoryConfig :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, maxRigidContactCount) == 4, "Wrong offset for PxgDynamicsMemoryConfig.maxRigidContactCount, expected 4 got %v", offset_of(PxgDynamicsMemoryConfig, maxRigidContactCount))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, maxRigidPatchCount) == 8, "Wrong offset for PxgDynamicsMemoryConfig.maxRigidPatchCount, expected 8 got %v", offset_of(PxgDynamicsMemoryConfig, maxRigidPatchCount))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, heapCapacity) == 12, "Wrong offset for PxgDynamicsMemoryConfig.heapCapacity, expected 12 got %v", offset_of(PxgDynamicsMemoryConfig, heapCapacity))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, foundLostPairsCapacity) == 16, "Wrong offset for PxgDynamicsMemoryConfig.foundLostPairsCapacity, expected 16 got %v", offset_of(PxgDynamicsMemoryConfig, foundLostPairsCapacity))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, foundLostAggregatePairsCapacity) == 20, "Wrong offset for PxgDynamicsMemoryConfig.foundLostAggregatePairsCapacity, expected 20 got %v", offset_of(PxgDynamicsMemoryConfig, foundLostAggregatePairsCapacity))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, totalAggregatePairsCapacity) == 24, "Wrong offset for PxgDynamicsMemoryConfig.totalAggregatePairsCapacity, expected 24 got %v", offset_of(PxgDynamicsMemoryConfig, totalAggregatePairsCapacity))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, maxSoftBodyContacts) == 28, "Wrong offset for PxgDynamicsMemoryConfig.maxSoftBodyContacts, expected 28 got %v", offset_of(PxgDynamicsMemoryConfig, maxSoftBodyContacts))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, maxFemClothContacts) == 32, "Wrong offset for PxgDynamicsMemoryConfig.maxFemClothContacts, expected 32 got %v", offset_of(PxgDynamicsMemoryConfig, maxFemClothContacts))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, maxParticleContacts) == 36, "Wrong offset for PxgDynamicsMemoryConfig.maxParticleContacts, expected 36 got %v", offset_of(PxgDynamicsMemoryConfig, maxParticleContacts))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, collisionStackSize) == 40, "Wrong offset for PxgDynamicsMemoryConfig.collisionStackSize, expected 40 got %v", offset_of(PxgDynamicsMemoryConfig, collisionStackSize))
    testing.expectf(t, offset_of(PxgDynamicsMemoryConfig, maxHairContacts) == 44, "Wrong offset for PxgDynamicsMemoryConfig.maxHairContacts, expected 44 got %v", offset_of(PxgDynamicsMemoryConfig, maxHairContacts))
    testing.expectf(t, size_of(PxgDynamicsMemoryConfig) == 48, "Wrong size for type PxgDynamicsMemoryConfig, expected 48 got %v", size_of(PxgDynamicsMemoryConfig))
}

@(test)
test_layout_PxSceneDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSceneDesc, gravity) == 36, "Wrong offset for PxSceneDesc.gravity, expected 36 got %v", offset_of(PxSceneDesc, gravity))
    testing.expectf(t, offset_of(PxSceneDesc, simulationEventCallback) == 48, "Wrong offset for PxSceneDesc.simulationEventCallback, expected 48 got %v", offset_of(PxSceneDesc, simulationEventCallback))
    testing.expectf(t, offset_of(PxSceneDesc, contactModifyCallback) == 56, "Wrong offset for PxSceneDesc.contactModifyCallback, expected 56 got %v", offset_of(PxSceneDesc, contactModifyCallback))
    testing.expectf(t, offset_of(PxSceneDesc, ccdContactModifyCallback) == 64, "Wrong offset for PxSceneDesc.ccdContactModifyCallback, expected 64 got %v", offset_of(PxSceneDesc, ccdContactModifyCallback))
    testing.expectf(t, offset_of(PxSceneDesc, filterShaderData) == 72, "Wrong offset for PxSceneDesc.filterShaderData, expected 72 got %v", offset_of(PxSceneDesc, filterShaderData))
    testing.expectf(t, offset_of(PxSceneDesc, filterShaderDataSize) == 80, "Wrong offset for PxSceneDesc.filterShaderDataSize, expected 80 got %v", offset_of(PxSceneDesc, filterShaderDataSize))
    testing.expectf(t, offset_of(PxSceneDesc, filterShader) == 88, "Wrong offset for PxSceneDesc.filterShader, expected 88 got %v", offset_of(PxSceneDesc, filterShader))
    testing.expectf(t, offset_of(PxSceneDesc, filterCallback) == 96, "Wrong offset for PxSceneDesc.filterCallback, expected 96 got %v", offset_of(PxSceneDesc, filterCallback))
    testing.expectf(t, offset_of(PxSceneDesc, kineKineFilteringMode) == 104, "Wrong offset for PxSceneDesc.kineKineFilteringMode, expected 104 got %v", offset_of(PxSceneDesc, kineKineFilteringMode))
    testing.expectf(t, offset_of(PxSceneDesc, staticKineFilteringMode) == 108, "Wrong offset for PxSceneDesc.staticKineFilteringMode, expected 108 got %v", offset_of(PxSceneDesc, staticKineFilteringMode))
    testing.expectf(t, offset_of(PxSceneDesc, broadPhaseType) == 112, "Wrong offset for PxSceneDesc.broadPhaseType, expected 112 got %v", offset_of(PxSceneDesc, broadPhaseType))
    testing.expectf(t, offset_of(PxSceneDesc, broadPhaseCallback) == 120, "Wrong offset for PxSceneDesc.broadPhaseCallback, expected 120 got %v", offset_of(PxSceneDesc, broadPhaseCallback))
    testing.expectf(t, offset_of(PxSceneDesc, limits) == 128, "Wrong offset for PxSceneDesc.limits, expected 128 got %v", offset_of(PxSceneDesc, limits))
    testing.expectf(t, offset_of(PxSceneDesc, frictionType) == 160, "Wrong offset for PxSceneDesc.frictionType, expected 160 got %v", offset_of(PxSceneDesc, frictionType))
    testing.expectf(t, offset_of(PxSceneDesc, solverType) == 164, "Wrong offset for PxSceneDesc.solverType, expected 164 got %v", offset_of(PxSceneDesc, solverType))
    testing.expectf(t, offset_of(PxSceneDesc, bounceThresholdVelocity) == 168, "Wrong offset for PxSceneDesc.bounceThresholdVelocity, expected 168 got %v", offset_of(PxSceneDesc, bounceThresholdVelocity))
    testing.expectf(t, offset_of(PxSceneDesc, frictionOffsetThreshold) == 172, "Wrong offset for PxSceneDesc.frictionOffsetThreshold, expected 172 got %v", offset_of(PxSceneDesc, frictionOffsetThreshold))
    testing.expectf(t, offset_of(PxSceneDesc, frictionCorrelationDistance) == 176, "Wrong offset for PxSceneDesc.frictionCorrelationDistance, expected 176 got %v", offset_of(PxSceneDesc, frictionCorrelationDistance))
    testing.expectf(t, offset_of(PxSceneDesc, flags) == 180, "Wrong offset for PxSceneDesc.flags, expected 180 got %v", offset_of(PxSceneDesc, flags))
    testing.expectf(t, offset_of(PxSceneDesc, cpuDispatcher) == 184, "Wrong offset for PxSceneDesc.cpuDispatcher, expected 184 got %v", offset_of(PxSceneDesc, cpuDispatcher))
    testing.expectf(t, offset_of(PxSceneDesc, userData) == 200, "Wrong offset for PxSceneDesc.userData, expected 200 got %v", offset_of(PxSceneDesc, userData))
    testing.expectf(t, offset_of(PxSceneDesc, solverBatchSize) == 208, "Wrong offset for PxSceneDesc.solverBatchSize, expected 208 got %v", offset_of(PxSceneDesc, solverBatchSize))
    testing.expectf(t, offset_of(PxSceneDesc, solverArticulationBatchSize) == 212, "Wrong offset for PxSceneDesc.solverArticulationBatchSize, expected 212 got %v", offset_of(PxSceneDesc, solverArticulationBatchSize))
    testing.expectf(t, offset_of(PxSceneDesc, nbContactDataBlocks) == 216, "Wrong offset for PxSceneDesc.nbContactDataBlocks, expected 216 got %v", offset_of(PxSceneDesc, nbContactDataBlocks))
    testing.expectf(t, offset_of(PxSceneDesc, maxNbContactDataBlocks) == 220, "Wrong offset for PxSceneDesc.maxNbContactDataBlocks, expected 220 got %v", offset_of(PxSceneDesc, maxNbContactDataBlocks))
    testing.expectf(t, offset_of(PxSceneDesc, maxBiasCoefficient) == 224, "Wrong offset for PxSceneDesc.maxBiasCoefficient, expected 224 got %v", offset_of(PxSceneDesc, maxBiasCoefficient))
    testing.expectf(t, offset_of(PxSceneDesc, contactReportStreamBufferSize) == 228, "Wrong offset for PxSceneDesc.contactReportStreamBufferSize, expected 228 got %v", offset_of(PxSceneDesc, contactReportStreamBufferSize))
    testing.expectf(t, offset_of(PxSceneDesc, ccdMaxPasses) == 232, "Wrong offset for PxSceneDesc.ccdMaxPasses, expected 232 got %v", offset_of(PxSceneDesc, ccdMaxPasses))
    testing.expectf(t, offset_of(PxSceneDesc, ccdThreshold) == 236, "Wrong offset for PxSceneDesc.ccdThreshold, expected 236 got %v", offset_of(PxSceneDesc, ccdThreshold))
    testing.expectf(t, offset_of(PxSceneDesc, ccdMaxSeparation) == 240, "Wrong offset for PxSceneDesc.ccdMaxSeparation, expected 240 got %v", offset_of(PxSceneDesc, ccdMaxSeparation))
    testing.expectf(t, offset_of(PxSceneDesc, wakeCounterResetValue) == 244, "Wrong offset for PxSceneDesc.wakeCounterResetValue, expected 244 got %v", offset_of(PxSceneDesc, wakeCounterResetValue))
    testing.expectf(t, offset_of(PxSceneDesc, sanityBounds) == 248, "Wrong offset for PxSceneDesc.sanityBounds, expected 248 got %v", offset_of(PxSceneDesc, sanityBounds))
    testing.expectf(t, offset_of(PxSceneDesc, gpuDynamicsConfig) == 272, "Wrong offset for PxSceneDesc.gpuDynamicsConfig, expected 272 got %v", offset_of(PxSceneDesc, gpuDynamicsConfig))
    testing.expectf(t, offset_of(PxSceneDesc, gpuMaxNumPartitions) == 320, "Wrong offset for PxSceneDesc.gpuMaxNumPartitions, expected 320 got %v", offset_of(PxSceneDesc, gpuMaxNumPartitions))
    testing.expectf(t, offset_of(PxSceneDesc, gpuMaxNumStaticPartitions) == 324, "Wrong offset for PxSceneDesc.gpuMaxNumStaticPartitions, expected 324 got %v", offset_of(PxSceneDesc, gpuMaxNumStaticPartitions))
    testing.expectf(t, offset_of(PxSceneDesc, gpuComputeVersion) == 328, "Wrong offset for PxSceneDesc.gpuComputeVersion, expected 328 got %v", offset_of(PxSceneDesc, gpuComputeVersion))
    testing.expectf(t, offset_of(PxSceneDesc, contactPairSlabSize) == 332, "Wrong offset for PxSceneDesc.contactPairSlabSize, expected 332 got %v", offset_of(PxSceneDesc, contactPairSlabSize))
    testing.expectf(t, offset_of(PxSceneDesc, sceneQuerySystem) == 336, "Wrong offset for PxSceneDesc.sceneQuerySystem, expected 336 got %v", offset_of(PxSceneDesc, sceneQuerySystem))
    testing.expectf(t, size_of(PxSceneDesc) == 352, "Wrong size for type PxSceneDesc, expected 352 got %v", size_of(PxSceneDesc))
}

@(test)
test_layout_PxSimulationStatistics :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSimulationStatistics, nbActiveDynamicBodies) == 4, "Wrong offset for PxSimulationStatistics.nbActiveDynamicBodies, expected 4 got %v", offset_of(PxSimulationStatistics, nbActiveDynamicBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbActiveKinematicBodies) == 8, "Wrong offset for PxSimulationStatistics.nbActiveKinematicBodies, expected 8 got %v", offset_of(PxSimulationStatistics, nbActiveKinematicBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbStaticBodies) == 12, "Wrong offset for PxSimulationStatistics.nbStaticBodies, expected 12 got %v", offset_of(PxSimulationStatistics, nbStaticBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbDynamicBodies) == 16, "Wrong offset for PxSimulationStatistics.nbDynamicBodies, expected 16 got %v", offset_of(PxSimulationStatistics, nbDynamicBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbKinematicBodies) == 20, "Wrong offset for PxSimulationStatistics.nbKinematicBodies, expected 20 got %v", offset_of(PxSimulationStatistics, nbKinematicBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbAggregates) == 68, "Wrong offset for PxSimulationStatistics.nbAggregates, expected 68 got %v", offset_of(PxSimulationStatistics, nbAggregates))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbArticulations) == 72, "Wrong offset for PxSimulationStatistics.nbArticulations, expected 72 got %v", offset_of(PxSimulationStatistics, nbArticulations))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbAxisSolverConstraints) == 76, "Wrong offset for PxSimulationStatistics.nbAxisSolverConstraints, expected 76 got %v", offset_of(PxSimulationStatistics, nbAxisSolverConstraints))
    testing.expectf(t, offset_of(PxSimulationStatistics, compressedContactSize) == 80, "Wrong offset for PxSimulationStatistics.compressedContactSize, expected 80 got %v", offset_of(PxSimulationStatistics, compressedContactSize))
    testing.expectf(t, offset_of(PxSimulationStatistics, requiredContactConstraintMemory) == 84, "Wrong offset for PxSimulationStatistics.requiredContactConstraintMemory, expected 84 got %v", offset_of(PxSimulationStatistics, requiredContactConstraintMemory))
    testing.expectf(t, offset_of(PxSimulationStatistics, peakConstraintMemory) == 88, "Wrong offset for PxSimulationStatistics.peakConstraintMemory, expected 88 got %v", offset_of(PxSimulationStatistics, peakConstraintMemory))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbDiscreteContactPairsTotal) == 92, "Wrong offset for PxSimulationStatistics.nbDiscreteContactPairsTotal, expected 92 got %v", offset_of(PxSimulationStatistics, nbDiscreteContactPairsTotal))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbDiscreteContactPairsWithCacheHits) == 96, "Wrong offset for PxSimulationStatistics.nbDiscreteContactPairsWithCacheHits, expected 96 got %v", offset_of(PxSimulationStatistics, nbDiscreteContactPairsWithCacheHits))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbDiscreteContactPairsWithContacts) == 100, "Wrong offset for PxSimulationStatistics.nbDiscreteContactPairsWithContacts, expected 100 got %v", offset_of(PxSimulationStatistics, nbDiscreteContactPairsWithContacts))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbNewPairs) == 104, "Wrong offset for PxSimulationStatistics.nbNewPairs, expected 104 got %v", offset_of(PxSimulationStatistics, nbNewPairs))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbLostPairs) == 108, "Wrong offset for PxSimulationStatistics.nbLostPairs, expected 108 got %v", offset_of(PxSimulationStatistics, nbLostPairs))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbNewTouches) == 112, "Wrong offset for PxSimulationStatistics.nbNewTouches, expected 112 got %v", offset_of(PxSimulationStatistics, nbNewTouches))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbLostTouches) == 116, "Wrong offset for PxSimulationStatistics.nbLostTouches, expected 116 got %v", offset_of(PxSimulationStatistics, nbLostTouches))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbPartitions) == 120, "Wrong offset for PxSimulationStatistics.nbPartitions, expected 120 got %v", offset_of(PxSimulationStatistics, nbPartitions))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemParticles) == 128, "Wrong offset for PxSimulationStatistics.gpuMemParticles, expected 128 got %v", offset_of(PxSimulationStatistics, gpuMemParticles))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemSoftBodies) == 136, "Wrong offset for PxSimulationStatistics.gpuMemSoftBodies, expected 136 got %v", offset_of(PxSimulationStatistics, gpuMemSoftBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemFEMCloths) == 144, "Wrong offset for PxSimulationStatistics.gpuMemFEMCloths, expected 144 got %v", offset_of(PxSimulationStatistics, gpuMemFEMCloths))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHairSystems) == 152, "Wrong offset for PxSimulationStatistics.gpuMemHairSystems, expected 152 got %v", offset_of(PxSimulationStatistics, gpuMemHairSystems))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeap) == 160, "Wrong offset for PxSimulationStatistics.gpuMemHeap, expected 160 got %v", offset_of(PxSimulationStatistics, gpuMemHeap))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapBroadPhase) == 168, "Wrong offset for PxSimulationStatistics.gpuMemHeapBroadPhase, expected 168 got %v", offset_of(PxSimulationStatistics, gpuMemHeapBroadPhase))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapNarrowPhase) == 176, "Wrong offset for PxSimulationStatistics.gpuMemHeapNarrowPhase, expected 176 got %v", offset_of(PxSimulationStatistics, gpuMemHeapNarrowPhase))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSolver) == 184, "Wrong offset for PxSimulationStatistics.gpuMemHeapSolver, expected 184 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSolver))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapArticulation) == 192, "Wrong offset for PxSimulationStatistics.gpuMemHeapArticulation, expected 192 got %v", offset_of(PxSimulationStatistics, gpuMemHeapArticulation))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSimulation) == 200, "Wrong offset for PxSimulationStatistics.gpuMemHeapSimulation, expected 200 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSimulation))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSimulationArticulation) == 208, "Wrong offset for PxSimulationStatistics.gpuMemHeapSimulationArticulation, expected 208 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSimulationArticulation))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSimulationParticles) == 216, "Wrong offset for PxSimulationStatistics.gpuMemHeapSimulationParticles, expected 216 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSimulationParticles))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSimulationSoftBody) == 224, "Wrong offset for PxSimulationStatistics.gpuMemHeapSimulationSoftBody, expected 224 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSimulationSoftBody))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSimulationFEMCloth) == 232, "Wrong offset for PxSimulationStatistics.gpuMemHeapSimulationFEMCloth, expected 232 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSimulationFEMCloth))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSimulationHairSystem) == 240, "Wrong offset for PxSimulationStatistics.gpuMemHeapSimulationHairSystem, expected 240 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSimulationHairSystem))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapParticles) == 248, "Wrong offset for PxSimulationStatistics.gpuMemHeapParticles, expected 248 got %v", offset_of(PxSimulationStatistics, gpuMemHeapParticles))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapSoftBodies) == 256, "Wrong offset for PxSimulationStatistics.gpuMemHeapSoftBodies, expected 256 got %v", offset_of(PxSimulationStatistics, gpuMemHeapSoftBodies))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapFEMCloths) == 264, "Wrong offset for PxSimulationStatistics.gpuMemHeapFEMCloths, expected 264 got %v", offset_of(PxSimulationStatistics, gpuMemHeapFEMCloths))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapHairSystems) == 272, "Wrong offset for PxSimulationStatistics.gpuMemHeapHairSystems, expected 272 got %v", offset_of(PxSimulationStatistics, gpuMemHeapHairSystems))
    testing.expectf(t, offset_of(PxSimulationStatistics, gpuMemHeapOther) == 280, "Wrong offset for PxSimulationStatistics.gpuMemHeapOther, expected 280 got %v", offset_of(PxSimulationStatistics, gpuMemHeapOther))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbBroadPhaseAdds) == 288, "Wrong offset for PxSimulationStatistics.nbBroadPhaseAdds, expected 288 got %v", offset_of(PxSimulationStatistics, nbBroadPhaseAdds))
    testing.expectf(t, offset_of(PxSimulationStatistics, nbBroadPhaseRemoves) == 292, "Wrong offset for PxSimulationStatistics.nbBroadPhaseRemoves, expected 292 got %v", offset_of(PxSimulationStatistics, nbBroadPhaseRemoves))
    testing.expectf(t, size_of(PxSimulationStatistics) == 2232, "Wrong size for type PxSimulationStatistics, expected 2232 got %v", size_of(PxSimulationStatistics))
}

@(test)
test_layout_PxGpuBodyData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGpuBodyData, pos) == 16, "Wrong offset for PxGpuBodyData.pos, expected 16 got %v", offset_of(PxGpuBodyData, pos))
    testing.expectf(t, offset_of(PxGpuBodyData, linVel) == 32, "Wrong offset for PxGpuBodyData.linVel, expected 32 got %v", offset_of(PxGpuBodyData, linVel))
    testing.expectf(t, offset_of(PxGpuBodyData, angVel) == 48, "Wrong offset for PxGpuBodyData.angVel, expected 48 got %v", offset_of(PxGpuBodyData, angVel))
    testing.expectf(t, size_of(PxGpuBodyData) == 64, "Wrong size for type PxGpuBodyData, expected 64 got %v", size_of(PxGpuBodyData))
}

@(test)
test_layout_PxGpuActorPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGpuActorPair, nodeIndex) == 8, "Wrong offset for PxGpuActorPair.nodeIndex, expected 8 got %v", offset_of(PxGpuActorPair, nodeIndex))
    testing.expectf(t, size_of(PxGpuActorPair) == 16, "Wrong size for type PxGpuActorPair, expected 16 got %v", size_of(PxGpuActorPair))
}

@(test)
test_layout_PxIndexDataPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxIndexDataPair, data) == 8, "Wrong offset for PxIndexDataPair.data, expected 8 got %v", offset_of(PxIndexDataPair, data))
    testing.expectf(t, size_of(PxIndexDataPair) == 16, "Wrong size for type PxIndexDataPair, expected 16 got %v", size_of(PxIndexDataPair))
}

@(test)
test_layout_PxPvdSceneClient :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPvdSceneClient) == 8, "Wrong size for type PxPvdSceneClient, expected 8 got %v", size_of(PxPvdSceneClient))
}

@(test)
test_layout_PxDominanceGroupPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDominanceGroupPair, dominance1) == 1, "Wrong offset for PxDominanceGroupPair.dominance1, expected 1 got %v", offset_of(PxDominanceGroupPair, dominance1))
    testing.expectf(t, size_of(PxDominanceGroupPair) == 2, "Wrong size for type PxDominanceGroupPair, expected 2 got %v", size_of(PxDominanceGroupPair))
}

@(test)
test_layout_PxBroadPhaseCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBroadPhaseCallback) == 8, "Wrong size for type PxBroadPhaseCallback, expected 8 got %v", size_of(PxBroadPhaseCallback))
}

@(test)
test_layout_PxScene :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxScene, userData) == 8, "Wrong offset for PxScene.userData, expected 8 got %v", offset_of(PxScene, userData))
    testing.expectf(t, size_of(PxScene) == 16, "Wrong size for type PxScene, expected 16 got %v", size_of(PxScene))
}

@(test)
test_layout_PxSceneReadLock :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSceneReadLock) == 8, "Wrong size for type PxSceneReadLock, expected 8 got %v", size_of(PxSceneReadLock))
}

@(test)
test_layout_PxSceneWriteLock :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSceneWriteLock) == 8, "Wrong size for type PxSceneWriteLock, expected 8 got %v", size_of(PxSceneWriteLock))
}

@(test)
test_layout_PxContactPairExtraDataItem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxContactPairExtraDataItem) == 1, "Wrong size for type PxContactPairExtraDataItem, expected 1 got %v", size_of(PxContactPairExtraDataItem))
}

@(test)
test_layout_PxContactPairVelocity :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxContactPairVelocity) == 52, "Wrong size for type PxContactPairVelocity, expected 52 got %v", size_of(PxContactPairVelocity))
}

@(test)
test_layout_PxContactPairPose :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxContactPairPose) == 60, "Wrong size for type PxContactPairPose, expected 60 got %v", size_of(PxContactPairPose))
}

@(test)
test_layout_PxContactPairIndex :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPairIndex, index) == 2, "Wrong offset for PxContactPairIndex.index, expected 2 got %v", offset_of(PxContactPairIndex, index))
    testing.expectf(t, size_of(PxContactPairIndex) == 4, "Wrong size for type PxContactPairIndex, expected 4 got %v", size_of(PxContactPairIndex))
}

@(test)
test_layout_PxContactPairExtraDataIterator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPairExtraDataIterator, endPtr) == 8, "Wrong offset for PxContactPairExtraDataIterator.endPtr, expected 8 got %v", offset_of(PxContactPairExtraDataIterator, endPtr))
    testing.expectf(t, offset_of(PxContactPairExtraDataIterator, preSolverVelocity) == 16, "Wrong offset for PxContactPairExtraDataIterator.preSolverVelocity, expected 16 got %v", offset_of(PxContactPairExtraDataIterator, preSolverVelocity))
    testing.expectf(t, offset_of(PxContactPairExtraDataIterator, postSolverVelocity) == 24, "Wrong offset for PxContactPairExtraDataIterator.postSolverVelocity, expected 24 got %v", offset_of(PxContactPairExtraDataIterator, postSolverVelocity))
    testing.expectf(t, offset_of(PxContactPairExtraDataIterator, eventPose) == 32, "Wrong offset for PxContactPairExtraDataIterator.eventPose, expected 32 got %v", offset_of(PxContactPairExtraDataIterator, eventPose))
    testing.expectf(t, offset_of(PxContactPairExtraDataIterator, contactPairIndex) == 40, "Wrong offset for PxContactPairExtraDataIterator.contactPairIndex, expected 40 got %v", offset_of(PxContactPairExtraDataIterator, contactPairIndex))
    testing.expectf(t, size_of(PxContactPairExtraDataIterator) == 48, "Wrong size for type PxContactPairExtraDataIterator, expected 48 got %v", size_of(PxContactPairExtraDataIterator))
}

@(test)
test_layout_PxContactPairHeader :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPairHeader, extraDataStream) == 16, "Wrong offset for PxContactPairHeader.extraDataStream, expected 16 got %v", offset_of(PxContactPairHeader, extraDataStream))
    testing.expectf(t, offset_of(PxContactPairHeader, extraDataStreamSize) == 24, "Wrong offset for PxContactPairHeader.extraDataStreamSize, expected 24 got %v", offset_of(PxContactPairHeader, extraDataStreamSize))
    testing.expectf(t, offset_of(PxContactPairHeader, flags) == 26, "Wrong offset for PxContactPairHeader.flags, expected 26 got %v", offset_of(PxContactPairHeader, flags))
    testing.expectf(t, offset_of(PxContactPairHeader, pairs) == 32, "Wrong offset for PxContactPairHeader.pairs, expected 32 got %v", offset_of(PxContactPairHeader, pairs))
    testing.expectf(t, offset_of(PxContactPairHeader, nbPairs) == 40, "Wrong offset for PxContactPairHeader.nbPairs, expected 40 got %v", offset_of(PxContactPairHeader, nbPairs))
    testing.expectf(t, size_of(PxContactPairHeader) == 48, "Wrong size for type PxContactPairHeader, expected 48 got %v", size_of(PxContactPairHeader))
}

@(test)
test_layout_PxContactPairPoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPairPoint, separation) == 12, "Wrong offset for PxContactPairPoint.separation, expected 12 got %v", offset_of(PxContactPairPoint, separation))
    testing.expectf(t, offset_of(PxContactPairPoint, normal) == 16, "Wrong offset for PxContactPairPoint.normal, expected 16 got %v", offset_of(PxContactPairPoint, normal))
    testing.expectf(t, offset_of(PxContactPairPoint, internalFaceIndex0) == 28, "Wrong offset for PxContactPairPoint.internalFaceIndex0, expected 28 got %v", offset_of(PxContactPairPoint, internalFaceIndex0))
    testing.expectf(t, offset_of(PxContactPairPoint, impulse) == 32, "Wrong offset for PxContactPairPoint.impulse, expected 32 got %v", offset_of(PxContactPairPoint, impulse))
    testing.expectf(t, offset_of(PxContactPairPoint, internalFaceIndex1) == 44, "Wrong offset for PxContactPairPoint.internalFaceIndex1, expected 44 got %v", offset_of(PxContactPairPoint, internalFaceIndex1))
    testing.expectf(t, size_of(PxContactPairPoint) == 48, "Wrong size for type PxContactPairPoint, expected 48 got %v", size_of(PxContactPairPoint))
}

@(test)
test_layout_PxContactPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxContactPair, contactPatches) == 16, "Wrong offset for PxContactPair.contactPatches, expected 16 got %v", offset_of(PxContactPair, contactPatches))
    testing.expectf(t, offset_of(PxContactPair, contactPoints) == 24, "Wrong offset for PxContactPair.contactPoints, expected 24 got %v", offset_of(PxContactPair, contactPoints))
    testing.expectf(t, offset_of(PxContactPair, contactImpulses) == 32, "Wrong offset for PxContactPair.contactImpulses, expected 32 got %v", offset_of(PxContactPair, contactImpulses))
    testing.expectf(t, offset_of(PxContactPair, requiredBufferSize) == 40, "Wrong offset for PxContactPair.requiredBufferSize, expected 40 got %v", offset_of(PxContactPair, requiredBufferSize))
    testing.expectf(t, offset_of(PxContactPair, contactCount) == 44, "Wrong offset for PxContactPair.contactCount, expected 44 got %v", offset_of(PxContactPair, contactCount))
    testing.expectf(t, offset_of(PxContactPair, patchCount) == 45, "Wrong offset for PxContactPair.patchCount, expected 45 got %v", offset_of(PxContactPair, patchCount))
    testing.expectf(t, offset_of(PxContactPair, contactStreamSize) == 46, "Wrong offset for PxContactPair.contactStreamSize, expected 46 got %v", offset_of(PxContactPair, contactStreamSize))
    testing.expectf(t, offset_of(PxContactPair, flags) == 48, "Wrong offset for PxContactPair.flags, expected 48 got %v", offset_of(PxContactPair, flags))
    testing.expectf(t, offset_of(PxContactPair, events) == 50, "Wrong offset for PxContactPair.events, expected 50 got %v", offset_of(PxContactPair, events))
    testing.expectf(t, size_of(PxContactPair) == 64, "Wrong size for type PxContactPair, expected 64 got %v", size_of(PxContactPair))
}

@(test)
test_layout_PxTriggerPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTriggerPair, triggerActor) == 8, "Wrong offset for PxTriggerPair.triggerActor, expected 8 got %v", offset_of(PxTriggerPair, triggerActor))
    testing.expectf(t, offset_of(PxTriggerPair, otherShape) == 16, "Wrong offset for PxTriggerPair.otherShape, expected 16 got %v", offset_of(PxTriggerPair, otherShape))
    testing.expectf(t, offset_of(PxTriggerPair, otherActor) == 24, "Wrong offset for PxTriggerPair.otherActor, expected 24 got %v", offset_of(PxTriggerPair, otherActor))
    testing.expectf(t, offset_of(PxTriggerPair, status) == 32, "Wrong offset for PxTriggerPair.status, expected 32 got %v", offset_of(PxTriggerPair, status))
    testing.expectf(t, offset_of(PxTriggerPair, flags) == 36, "Wrong offset for PxTriggerPair.flags, expected 36 got %v", offset_of(PxTriggerPair, flags))
    testing.expectf(t, size_of(PxTriggerPair) == 40, "Wrong size for type PxTriggerPair, expected 40 got %v", size_of(PxTriggerPair))
}

@(test)
test_layout_PxConstraintInfo :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConstraintInfo, externalReference) == 8, "Wrong offset for PxConstraintInfo.externalReference, expected 8 got %v", offset_of(PxConstraintInfo, externalReference))
    testing.expectf(t, offset_of(PxConstraintInfo, type) == 16, "Wrong offset for PxConstraintInfo.type, expected 16 got %v", offset_of(PxConstraintInfo, type))
    testing.expectf(t, size_of(PxConstraintInfo) == 24, "Wrong size for type PxConstraintInfo, expected 24 got %v", size_of(PxConstraintInfo))
}

@(test)
test_layout_PxSimulationEventCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSimulationEventCallback) == 8, "Wrong size for type PxSimulationEventCallback, expected 8 got %v", size_of(PxSimulationEventCallback))
}

@(test)
test_layout_PxFEMParameters :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxFEMParameters, settlingThreshold) == 4, "Wrong offset for PxFEMParameters.settlingThreshold, expected 4 got %v", offset_of(PxFEMParameters, settlingThreshold))
    testing.expectf(t, offset_of(PxFEMParameters, sleepThreshold) == 8, "Wrong offset for PxFEMParameters.sleepThreshold, expected 8 got %v", offset_of(PxFEMParameters, sleepThreshold))
    testing.expectf(t, offset_of(PxFEMParameters, sleepDamping) == 12, "Wrong offset for PxFEMParameters.sleepDamping, expected 12 got %v", offset_of(PxFEMParameters, sleepDamping))
    testing.expectf(t, offset_of(PxFEMParameters, selfCollisionFilterDistance) == 16, "Wrong offset for PxFEMParameters.selfCollisionFilterDistance, expected 16 got %v", offset_of(PxFEMParameters, selfCollisionFilterDistance))
    testing.expectf(t, offset_of(PxFEMParameters, selfCollisionStressTolerance) == 20, "Wrong offset for PxFEMParameters.selfCollisionStressTolerance, expected 20 got %v", offset_of(PxFEMParameters, selfCollisionStressTolerance))
    testing.expectf(t, size_of(PxFEMParameters) == 24, "Wrong size for type PxFEMParameters, expected 24 got %v", size_of(PxFEMParameters))
}

@(test)
test_layout_PxPruningStructure :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPruningStructure) == 16, "Wrong size for type PxPruningStructure, expected 16 got %v", size_of(PxPruningStructure))
}

@(test)
test_layout_PxExtendedVec3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxExtendedVec3, y) == 8, "Wrong offset for PxExtendedVec3.y, expected 8 got %v", offset_of(PxExtendedVec3, y))
    testing.expectf(t, offset_of(PxExtendedVec3, z) == 16, "Wrong offset for PxExtendedVec3.z, expected 16 got %v", offset_of(PxExtendedVec3, z))
    testing.expectf(t, size_of(PxExtendedVec3) == 24, "Wrong size for type PxExtendedVec3, expected 24 got %v", size_of(PxExtendedVec3))
}

@(test)
test_layout_PxObstacle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxObstacle, mUserData) == 8, "Wrong offset for PxObstacle.mUserData, expected 8 got %v", offset_of(PxObstacle, mUserData))
    testing.expectf(t, offset_of(PxObstacle, mPos) == 16, "Wrong offset for PxObstacle.mPos, expected 16 got %v", offset_of(PxObstacle, mPos))
    testing.expectf(t, offset_of(PxObstacle, mRot) == 40, "Wrong offset for PxObstacle.mRot, expected 40 got %v", offset_of(PxObstacle, mRot))
    testing.expectf(t, size_of(PxObstacle) == 56, "Wrong size for type PxObstacle, expected 56 got %v", size_of(PxObstacle))
}

@(test)
test_layout_PxBoxObstacle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBoxObstacle, mHalfExtents) == 56, "Wrong offset for PxBoxObstacle.mHalfExtents, expected 56 got %v", offset_of(PxBoxObstacle, mHalfExtents))
    testing.expectf(t, size_of(PxBoxObstacle) == 72, "Wrong size for type PxBoxObstacle, expected 72 got %v", size_of(PxBoxObstacle))
}

@(test)
test_layout_PxCapsuleObstacle :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxCapsuleObstacle, mHalfHeight) == 56, "Wrong offset for PxCapsuleObstacle.mHalfHeight, expected 56 got %v", offset_of(PxCapsuleObstacle, mHalfHeight))
    testing.expectf(t, offset_of(PxCapsuleObstacle, mRadius) == 60, "Wrong offset for PxCapsuleObstacle.mRadius, expected 60 got %v", offset_of(PxCapsuleObstacle, mRadius))
    testing.expectf(t, size_of(PxCapsuleObstacle) == 64, "Wrong size for type PxCapsuleObstacle, expected 64 got %v", size_of(PxCapsuleObstacle))
}

@(test)
test_layout_PxObstacleContext :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxObstacleContext) == 8, "Wrong size for type PxObstacleContext, expected 8 got %v", size_of(PxObstacleContext))
}

@(test)
test_layout_PxControllerState :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerState, touchedShape) == 16, "Wrong offset for PxControllerState.touchedShape, expected 16 got %v", offset_of(PxControllerState, touchedShape))
    testing.expectf(t, offset_of(PxControllerState, touchedActor) == 24, "Wrong offset for PxControllerState.touchedActor, expected 24 got %v", offset_of(PxControllerState, touchedActor))
    testing.expectf(t, offset_of(PxControllerState, touchedObstacleHandle) == 32, "Wrong offset for PxControllerState.touchedObstacleHandle, expected 32 got %v", offset_of(PxControllerState, touchedObstacleHandle))
    testing.expectf(t, offset_of(PxControllerState, collisionFlags) == 36, "Wrong offset for PxControllerState.collisionFlags, expected 36 got %v", offset_of(PxControllerState, collisionFlags))
    testing.expectf(t, offset_of(PxControllerState, standOnAnotherCCT) == 40, "Wrong offset for PxControllerState.standOnAnotherCCT, expected 40 got %v", offset_of(PxControllerState, standOnAnotherCCT))
    testing.expectf(t, offset_of(PxControllerState, standOnObstacle) == 41, "Wrong offset for PxControllerState.standOnObstacle, expected 41 got %v", offset_of(PxControllerState, standOnObstacle))
    testing.expectf(t, offset_of(PxControllerState, isMovingUp) == 42, "Wrong offset for PxControllerState.isMovingUp, expected 42 got %v", offset_of(PxControllerState, isMovingUp))
    testing.expectf(t, size_of(PxControllerState) == 48, "Wrong size for type PxControllerState, expected 48 got %v", size_of(PxControllerState))
}

@(test)
test_layout_PxControllerStats :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerStats, nbFullUpdates) == 2, "Wrong offset for PxControllerStats.nbFullUpdates, expected 2 got %v", offset_of(PxControllerStats, nbFullUpdates))
    testing.expectf(t, offset_of(PxControllerStats, nbPartialUpdates) == 4, "Wrong offset for PxControllerStats.nbPartialUpdates, expected 4 got %v", offset_of(PxControllerStats, nbPartialUpdates))
    testing.expectf(t, offset_of(PxControllerStats, nbTessellation) == 6, "Wrong offset for PxControllerStats.nbTessellation, expected 6 got %v", offset_of(PxControllerStats, nbTessellation))
    testing.expectf(t, size_of(PxControllerStats) == 8, "Wrong size for type PxControllerStats, expected 8 got %v", size_of(PxControllerStats))
}

@(test)
test_layout_PxControllerHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerHit, worldPos) == 8, "Wrong offset for PxControllerHit.worldPos, expected 8 got %v", offset_of(PxControllerHit, worldPos))
    testing.expectf(t, offset_of(PxControllerHit, worldNormal) == 32, "Wrong offset for PxControllerHit.worldNormal, expected 32 got %v", offset_of(PxControllerHit, worldNormal))
    testing.expectf(t, offset_of(PxControllerHit, dir) == 44, "Wrong offset for PxControllerHit.dir, expected 44 got %v", offset_of(PxControllerHit, dir))
    testing.expectf(t, offset_of(PxControllerHit, length) == 56, "Wrong offset for PxControllerHit.length, expected 56 got %v", offset_of(PxControllerHit, length))
    testing.expectf(t, size_of(PxControllerHit) == 64, "Wrong size for type PxControllerHit, expected 64 got %v", size_of(PxControllerHit))
}

@(test)
test_layout_PxControllerShapeHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerShapeHit, shape) == 64, "Wrong offset for PxControllerShapeHit.shape, expected 64 got %v", offset_of(PxControllerShapeHit, shape))
    testing.expectf(t, offset_of(PxControllerShapeHit, actor) == 72, "Wrong offset for PxControllerShapeHit.actor, expected 72 got %v", offset_of(PxControllerShapeHit, actor))
    testing.expectf(t, offset_of(PxControllerShapeHit, triangleIndex) == 80, "Wrong offset for PxControllerShapeHit.triangleIndex, expected 80 got %v", offset_of(PxControllerShapeHit, triangleIndex))
    testing.expectf(t, size_of(PxControllerShapeHit) == 88, "Wrong size for type PxControllerShapeHit, expected 88 got %v", size_of(PxControllerShapeHit))
}

@(test)
test_layout_PxControllersHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllersHit, other) == 64, "Wrong offset for PxControllersHit.other, expected 64 got %v", offset_of(PxControllersHit, other))
    testing.expectf(t, size_of(PxControllersHit) == 72, "Wrong size for type PxControllersHit, expected 72 got %v", size_of(PxControllersHit))
}

@(test)
test_layout_PxControllerObstacleHit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerObstacleHit, userData) == 64, "Wrong offset for PxControllerObstacleHit.userData, expected 64 got %v", offset_of(PxControllerObstacleHit, userData))
    testing.expectf(t, size_of(PxControllerObstacleHit) == 72, "Wrong size for type PxControllerObstacleHit, expected 72 got %v", size_of(PxControllerObstacleHit))
}

@(test)
test_layout_PxUserControllerHitReport :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxUserControllerHitReport) == 8, "Wrong size for type PxUserControllerHitReport, expected 8 got %v", size_of(PxUserControllerHitReport))
}

@(test)
test_layout_PxControllerFilterCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxControllerFilterCallback) == 8, "Wrong size for type PxControllerFilterCallback, expected 8 got %v", size_of(PxControllerFilterCallback))
}

@(test)
test_layout_PxControllerFilters :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerFilters, mFilterCallback) == 8, "Wrong offset for PxControllerFilters.mFilterCallback, expected 8 got %v", offset_of(PxControllerFilters, mFilterCallback))
    testing.expectf(t, offset_of(PxControllerFilters, mFilterFlags) == 16, "Wrong offset for PxControllerFilters.mFilterFlags, expected 16 got %v", offset_of(PxControllerFilters, mFilterFlags))
    testing.expectf(t, offset_of(PxControllerFilters, mCCTFilterCallback) == 24, "Wrong offset for PxControllerFilters.mCCTFilterCallback, expected 24 got %v", offset_of(PxControllerFilters, mCCTFilterCallback))
    testing.expectf(t, size_of(PxControllerFilters) == 32, "Wrong size for type PxControllerFilters, expected 32 got %v", size_of(PxControllerFilters))
}

@(test)
test_layout_PxControllerDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxControllerDesc, position) == 8, "Wrong offset for PxControllerDesc.position, expected 8 got %v", offset_of(PxControllerDesc, position))
    testing.expectf(t, offset_of(PxControllerDesc, upDirection) == 32, "Wrong offset for PxControllerDesc.upDirection, expected 32 got %v", offset_of(PxControllerDesc, upDirection))
    testing.expectf(t, offset_of(PxControllerDesc, slopeLimit) == 44, "Wrong offset for PxControllerDesc.slopeLimit, expected 44 got %v", offset_of(PxControllerDesc, slopeLimit))
    testing.expectf(t, offset_of(PxControllerDesc, invisibleWallHeight) == 48, "Wrong offset for PxControllerDesc.invisibleWallHeight, expected 48 got %v", offset_of(PxControllerDesc, invisibleWallHeight))
    testing.expectf(t, offset_of(PxControllerDesc, maxJumpHeight) == 52, "Wrong offset for PxControllerDesc.maxJumpHeight, expected 52 got %v", offset_of(PxControllerDesc, maxJumpHeight))
    testing.expectf(t, offset_of(PxControllerDesc, contactOffset) == 56, "Wrong offset for PxControllerDesc.contactOffset, expected 56 got %v", offset_of(PxControllerDesc, contactOffset))
    testing.expectf(t, offset_of(PxControllerDesc, stepOffset) == 60, "Wrong offset for PxControllerDesc.stepOffset, expected 60 got %v", offset_of(PxControllerDesc, stepOffset))
    testing.expectf(t, offset_of(PxControllerDesc, density) == 64, "Wrong offset for PxControllerDesc.density, expected 64 got %v", offset_of(PxControllerDesc, density))
    testing.expectf(t, offset_of(PxControllerDesc, scaleCoeff) == 68, "Wrong offset for PxControllerDesc.scaleCoeff, expected 68 got %v", offset_of(PxControllerDesc, scaleCoeff))
    testing.expectf(t, offset_of(PxControllerDesc, volumeGrowth) == 72, "Wrong offset for PxControllerDesc.volumeGrowth, expected 72 got %v", offset_of(PxControllerDesc, volumeGrowth))
    testing.expectf(t, offset_of(PxControllerDesc, reportCallback) == 80, "Wrong offset for PxControllerDesc.reportCallback, expected 80 got %v", offset_of(PxControllerDesc, reportCallback))
    testing.expectf(t, offset_of(PxControllerDesc, behaviorCallback) == 88, "Wrong offset for PxControllerDesc.behaviorCallback, expected 88 got %v", offset_of(PxControllerDesc, behaviorCallback))
    testing.expectf(t, offset_of(PxControllerDesc, nonWalkableMode) == 96, "Wrong offset for PxControllerDesc.nonWalkableMode, expected 96 got %v", offset_of(PxControllerDesc, nonWalkableMode))
    testing.expectf(t, offset_of(PxControllerDesc, material) == 104, "Wrong offset for PxControllerDesc.material, expected 104 got %v", offset_of(PxControllerDesc, material))
    testing.expectf(t, offset_of(PxControllerDesc, registerDeletionListener) == 112, "Wrong offset for PxControllerDesc.registerDeletionListener, expected 112 got %v", offset_of(PxControllerDesc, registerDeletionListener))
    testing.expectf(t, offset_of(PxControllerDesc, clientID) == 113, "Wrong offset for PxControllerDesc.clientID, expected 113 got %v", offset_of(PxControllerDesc, clientID))
    testing.expectf(t, offset_of(PxControllerDesc, userData) == 120, "Wrong offset for PxControllerDesc.userData, expected 120 got %v", offset_of(PxControllerDesc, userData))
    testing.expectf(t, size_of(PxControllerDesc) == 132, "Wrong size for type PxControllerDesc, expected 132 got %v", size_of(PxControllerDesc))
}

@(test)
test_layout_PxController :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxController) == 8, "Wrong size for type PxController, expected 8 got %v", size_of(PxController))
}

@(test)
test_layout_PxBoxControllerDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBoxControllerDesc, halfHeight) == 132, "Wrong offset for PxBoxControllerDesc.halfHeight, expected 132 got %v", offset_of(PxBoxControllerDesc, halfHeight))
    testing.expectf(t, offset_of(PxBoxControllerDesc, halfSideExtent) == 136, "Wrong offset for PxBoxControllerDesc.halfSideExtent, expected 136 got %v", offset_of(PxBoxControllerDesc, halfSideExtent))
    testing.expectf(t, offset_of(PxBoxControllerDesc, halfForwardExtent) == 140, "Wrong offset for PxBoxControllerDesc.halfForwardExtent, expected 140 got %v", offset_of(PxBoxControllerDesc, halfForwardExtent))
    testing.expectf(t, size_of(PxBoxControllerDesc) == 144, "Wrong size for type PxBoxControllerDesc, expected 144 got %v", size_of(PxBoxControllerDesc))
}

@(test)
test_layout_PxBoxController :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBoxController) == 8, "Wrong size for type PxBoxController, expected 8 got %v", size_of(PxBoxController))
}

@(test)
test_layout_PxCapsuleControllerDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxCapsuleControllerDesc, radius) == 132, "Wrong offset for PxCapsuleControllerDesc.radius, expected 132 got %v", offset_of(PxCapsuleControllerDesc, radius))
    testing.expectf(t, offset_of(PxCapsuleControllerDesc, height) == 136, "Wrong offset for PxCapsuleControllerDesc.height, expected 136 got %v", offset_of(PxCapsuleControllerDesc, height))
    testing.expectf(t, offset_of(PxCapsuleControllerDesc, climbingMode) == 140, "Wrong offset for PxCapsuleControllerDesc.climbingMode, expected 140 got %v", offset_of(PxCapsuleControllerDesc, climbingMode))
    testing.expectf(t, size_of(PxCapsuleControllerDesc) == 144, "Wrong size for type PxCapsuleControllerDesc, expected 144 got %v", size_of(PxCapsuleControllerDesc))
}

@(test)
test_layout_PxCapsuleController :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCapsuleController) == 8, "Wrong size for type PxCapsuleController, expected 8 got %v", size_of(PxCapsuleController))
}

@(test)
test_layout_PxControllerBehaviorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxControllerBehaviorCallback) == 8, "Wrong size for type PxControllerBehaviorCallback, expected 8 got %v", size_of(PxControllerBehaviorCallback))
}

@(test)
test_layout_PxControllerManager :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxControllerManager) == 8, "Wrong size for type PxControllerManager, expected 8 got %v", size_of(PxControllerManager))
}

@(test)
test_layout_PxDim3 :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxDim3, y) == 4, "Wrong offset for PxDim3.y, expected 4 got %v", offset_of(PxDim3, y))
    testing.expectf(t, offset_of(PxDim3, z) == 8, "Wrong offset for PxDim3.z, expected 8 got %v", offset_of(PxDim3, z))
    testing.expectf(t, size_of(PxDim3) == 12, "Wrong size for type PxDim3, expected 12 got %v", size_of(PxDim3))
}

@(test)
test_layout_PxSDFDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSDFDesc, dims) == 24, "Wrong offset for PxSDFDesc.dims, expected 24 got %v", offset_of(PxSDFDesc, dims))
    testing.expectf(t, offset_of(PxSDFDesc, meshLower) == 36, "Wrong offset for PxSDFDesc.meshLower, expected 36 got %v", offset_of(PxSDFDesc, meshLower))
    testing.expectf(t, offset_of(PxSDFDesc, spacing) == 48, "Wrong offset for PxSDFDesc.spacing, expected 48 got %v", offset_of(PxSDFDesc, spacing))
    testing.expectf(t, offset_of(PxSDFDesc, subgridSize) == 52, "Wrong offset for PxSDFDesc.subgridSize, expected 52 got %v", offset_of(PxSDFDesc, subgridSize))
    testing.expectf(t, offset_of(PxSDFDesc, bitsPerSubgridPixel) == 56, "Wrong offset for PxSDFDesc.bitsPerSubgridPixel, expected 56 got %v", offset_of(PxSDFDesc, bitsPerSubgridPixel))
    testing.expectf(t, offset_of(PxSDFDesc, sdfSubgrids3DTexBlockDim) == 60, "Wrong offset for PxSDFDesc.sdfSubgrids3DTexBlockDim, expected 60 got %v", offset_of(PxSDFDesc, sdfSubgrids3DTexBlockDim))
    testing.expectf(t, offset_of(PxSDFDesc, sdfSubgrids) == 72, "Wrong offset for PxSDFDesc.sdfSubgrids, expected 72 got %v", offset_of(PxSDFDesc, sdfSubgrids))
    testing.expectf(t, offset_of(PxSDFDesc, sdfStartSlots) == 96, "Wrong offset for PxSDFDesc.sdfStartSlots, expected 96 got %v", offset_of(PxSDFDesc, sdfStartSlots))
    testing.expectf(t, offset_of(PxSDFDesc, subgridsMinSdfValue) == 120, "Wrong offset for PxSDFDesc.subgridsMinSdfValue, expected 120 got %v", offset_of(PxSDFDesc, subgridsMinSdfValue))
    testing.expectf(t, offset_of(PxSDFDesc, subgridsMaxSdfValue) == 124, "Wrong offset for PxSDFDesc.subgridsMaxSdfValue, expected 124 got %v", offset_of(PxSDFDesc, subgridsMaxSdfValue))
    testing.expectf(t, offset_of(PxSDFDesc, sdfBounds) == 128, "Wrong offset for PxSDFDesc.sdfBounds, expected 128 got %v", offset_of(PxSDFDesc, sdfBounds))
    testing.expectf(t, offset_of(PxSDFDesc, narrowBandThicknessRelativeToSdfBoundsDiagonal) == 152, "Wrong offset for PxSDFDesc.narrowBandThicknessRelativeToSdfBoundsDiagonal, expected 152 got %v", offset_of(PxSDFDesc, narrowBandThicknessRelativeToSdfBoundsDiagonal))
    testing.expectf(t, offset_of(PxSDFDesc, numThreadsForSdfConstruction) == 156, "Wrong offset for PxSDFDesc.numThreadsForSdfConstruction, expected 156 got %v", offset_of(PxSDFDesc, numThreadsForSdfConstruction))
    testing.expectf(t, size_of(PxSDFDesc) == 160, "Wrong size for type PxSDFDesc, expected 160 got %v", size_of(PxSDFDesc))
}

@(test)
test_layout_PxConvexMeshDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxConvexMeshDesc, polygons) == 24, "Wrong offset for PxConvexMeshDesc.polygons, expected 24 got %v", offset_of(PxConvexMeshDesc, polygons))
    testing.expectf(t, offset_of(PxConvexMeshDesc, indices) == 48, "Wrong offset for PxConvexMeshDesc.indices, expected 48 got %v", offset_of(PxConvexMeshDesc, indices))
    testing.expectf(t, offset_of(PxConvexMeshDesc, flags) == 72, "Wrong offset for PxConvexMeshDesc.flags, expected 72 got %v", offset_of(PxConvexMeshDesc, flags))
    testing.expectf(t, offset_of(PxConvexMeshDesc, vertexLimit) == 74, "Wrong offset for PxConvexMeshDesc.vertexLimit, expected 74 got %v", offset_of(PxConvexMeshDesc, vertexLimit))
    testing.expectf(t, offset_of(PxConvexMeshDesc, polygonLimit) == 76, "Wrong offset for PxConvexMeshDesc.polygonLimit, expected 76 got %v", offset_of(PxConvexMeshDesc, polygonLimit))
    testing.expectf(t, offset_of(PxConvexMeshDesc, quantizedCount) == 78, "Wrong offset for PxConvexMeshDesc.quantizedCount, expected 78 got %v", offset_of(PxConvexMeshDesc, quantizedCount))
    testing.expectf(t, offset_of(PxConvexMeshDesc, sdfDesc) == 80, "Wrong offset for PxConvexMeshDesc.sdfDesc, expected 80 got %v", offset_of(PxConvexMeshDesc, sdfDesc))
    testing.expectf(t, size_of(PxConvexMeshDesc) == 88, "Wrong size for type PxConvexMeshDesc, expected 88 got %v", size_of(PxConvexMeshDesc))
}

@(test)
test_layout_PxTriangleMeshDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTriangleMeshDesc, sdfDesc) == 72, "Wrong offset for PxTriangleMeshDesc.sdfDesc, expected 72 got %v", offset_of(PxTriangleMeshDesc, sdfDesc))
    testing.expectf(t, size_of(PxTriangleMeshDesc) == 80, "Wrong size for type PxTriangleMeshDesc, expected 80 got %v", size_of(PxTriangleMeshDesc))
}

@(test)
test_layout_PxTetrahedronMeshDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxTetrahedronMeshDesc, points) == 16, "Wrong offset for PxTetrahedronMeshDesc.points, expected 16 got %v", offset_of(PxTetrahedronMeshDesc, points))
    testing.expectf(t, offset_of(PxTetrahedronMeshDesc, tetrahedrons) == 40, "Wrong offset for PxTetrahedronMeshDesc.tetrahedrons, expected 40 got %v", offset_of(PxTetrahedronMeshDesc, tetrahedrons))
    testing.expectf(t, offset_of(PxTetrahedronMeshDesc, flags) == 64, "Wrong offset for PxTetrahedronMeshDesc.flags, expected 64 got %v", offset_of(PxTetrahedronMeshDesc, flags))
    testing.expectf(t, offset_of(PxTetrahedronMeshDesc, tetsPerElement) == 66, "Wrong offset for PxTetrahedronMeshDesc.tetsPerElement, expected 66 got %v", offset_of(PxTetrahedronMeshDesc, tetsPerElement))
    testing.expectf(t, size_of(PxTetrahedronMeshDesc) == 72, "Wrong size for type PxTetrahedronMeshDesc, expected 72 got %v", size_of(PxTetrahedronMeshDesc))
}

@(test)
test_layout_PxSoftBodySimulationDataDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSoftBodySimulationDataDesc) == 24, "Wrong size for type PxSoftBodySimulationDataDesc, expected 24 got %v", size_of(PxSoftBodySimulationDataDesc))
}

@(test)
test_layout_PxBVH34MidphaseDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBVH34MidphaseDesc, buildStrategy) == 4, "Wrong offset for PxBVH34MidphaseDesc.buildStrategy, expected 4 got %v", offset_of(PxBVH34MidphaseDesc, buildStrategy))
    testing.expectf(t, offset_of(PxBVH34MidphaseDesc, quantized) == 8, "Wrong offset for PxBVH34MidphaseDesc.quantized, expected 8 got %v", offset_of(PxBVH34MidphaseDesc, quantized))
    testing.expectf(t, size_of(PxBVH34MidphaseDesc) == 12, "Wrong size for type PxBVH34MidphaseDesc, expected 12 got %v", size_of(PxBVH34MidphaseDesc))
}

@(test)
test_layout_PxMidphaseDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxMidphaseDesc) == 16, "Wrong size for type PxMidphaseDesc, expected 16 got %v", size_of(PxMidphaseDesc))
}

@(test)
test_layout_PxBVHDesc :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxBVHDesc, enlargement) == 24, "Wrong offset for PxBVHDesc.enlargement, expected 24 got %v", offset_of(PxBVHDesc, enlargement))
    testing.expectf(t, offset_of(PxBVHDesc, numPrimsPerLeaf) == 28, "Wrong offset for PxBVHDesc.numPrimsPerLeaf, expected 28 got %v", offset_of(PxBVHDesc, numPrimsPerLeaf))
    testing.expectf(t, offset_of(PxBVHDesc, buildStrategy) == 32, "Wrong offset for PxBVHDesc.buildStrategy, expected 32 got %v", offset_of(PxBVHDesc, buildStrategy))
    testing.expectf(t, size_of(PxBVHDesc) == 40, "Wrong size for type PxBVHDesc, expected 40 got %v", size_of(PxBVHDesc))
}

@(test)
test_layout_PxCookingParams :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxCookingParams, planeTolerance) == 4, "Wrong offset for PxCookingParams.planeTolerance, expected 4 got %v", offset_of(PxCookingParams, planeTolerance))
    testing.expectf(t, offset_of(PxCookingParams, convexMeshCookingType) == 8, "Wrong offset for PxCookingParams.convexMeshCookingType, expected 8 got %v", offset_of(PxCookingParams, convexMeshCookingType))
    testing.expectf(t, offset_of(PxCookingParams, suppressTriangleMeshRemapTable) == 12, "Wrong offset for PxCookingParams.suppressTriangleMeshRemapTable, expected 12 got %v", offset_of(PxCookingParams, suppressTriangleMeshRemapTable))
    testing.expectf(t, offset_of(PxCookingParams, buildTriangleAdjacencies) == 13, "Wrong offset for PxCookingParams.buildTriangleAdjacencies, expected 13 got %v", offset_of(PxCookingParams, buildTriangleAdjacencies))
    testing.expectf(t, offset_of(PxCookingParams, buildGPUData) == 14, "Wrong offset for PxCookingParams.buildGPUData, expected 14 got %v", offset_of(PxCookingParams, buildGPUData))
    testing.expectf(t, offset_of(PxCookingParams, scale) == 16, "Wrong offset for PxCookingParams.scale, expected 16 got %v", offset_of(PxCookingParams, scale))
    testing.expectf(t, offset_of(PxCookingParams, meshPreprocessParams) == 24, "Wrong offset for PxCookingParams.meshPreprocessParams, expected 24 got %v", offset_of(PxCookingParams, meshPreprocessParams))
    testing.expectf(t, offset_of(PxCookingParams, meshWeldTolerance) == 28, "Wrong offset for PxCookingParams.meshWeldTolerance, expected 28 got %v", offset_of(PxCookingParams, meshWeldTolerance))
    testing.expectf(t, offset_of(PxCookingParams, midphaseDesc) == 32, "Wrong offset for PxCookingParams.midphaseDesc, expected 32 got %v", offset_of(PxCookingParams, midphaseDesc))
    testing.expectf(t, offset_of(PxCookingParams, gaussMapLimit) == 48, "Wrong offset for PxCookingParams.gaussMapLimit, expected 48 got %v", offset_of(PxCookingParams, gaussMapLimit))
    testing.expectf(t, offset_of(PxCookingParams, maxWeightRatioInTet) == 52, "Wrong offset for PxCookingParams.maxWeightRatioInTet, expected 52 got %v", offset_of(PxCookingParams, maxWeightRatioInTet))
    testing.expectf(t, size_of(PxCookingParams) == 56, "Wrong size for type PxCookingParams, expected 56 got %v", size_of(PxCookingParams))
}

@(test)
test_layout_PxDefaultMemoryOutputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultMemoryOutputStream) == 32, "Wrong size for type PxDefaultMemoryOutputStream, expected 32 got %v", size_of(PxDefaultMemoryOutputStream))
}

@(test)
test_layout_PxDefaultMemoryInputData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultMemoryInputData) == 32, "Wrong size for type PxDefaultMemoryInputData, expected 32 got %v", size_of(PxDefaultMemoryInputData))
}

@(test)
test_layout_PxDefaultFileOutputStream :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultFileOutputStream) == 16, "Wrong size for type PxDefaultFileOutputStream, expected 16 got %v", size_of(PxDefaultFileOutputStream))
}

@(test)
test_layout_PxDefaultFileInputData :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultFileInputData) == 24, "Wrong size for type PxDefaultFileInputData, expected 24 got %v", size_of(PxDefaultFileInputData))
}

@(test)
test_layout_PxDefaultAllocator :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultAllocator) == 8, "Wrong size for type PxDefaultAllocator, expected 8 got %v", size_of(PxDefaultAllocator))
}

@(test)
test_layout_PxJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJoint, userData) == 16, "Wrong offset for PxJoint.userData, expected 16 got %v", offset_of(PxJoint, userData))
    testing.expectf(t, size_of(PxJoint) == 24, "Wrong size for type PxJoint, expected 24 got %v", size_of(PxJoint))
}

@(test)
test_layout_PxSpring :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxSpring, damping) == 4, "Wrong offset for PxSpring.damping, expected 4 got %v", offset_of(PxSpring, damping))
    testing.expectf(t, size_of(PxSpring) == 8, "Wrong size for type PxSpring, expected 8 got %v", size_of(PxSpring))
}

@(test)
test_layout_PxDistanceJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDistanceJoint) == 24, "Wrong size for type PxDistanceJoint, expected 24 got %v", size_of(PxDistanceJoint))
}

@(test)
test_layout_PxJacobianRow :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJacobianRow, linear1) == 12, "Wrong offset for PxJacobianRow.linear1, expected 12 got %v", offset_of(PxJacobianRow, linear1))
    testing.expectf(t, offset_of(PxJacobianRow, angular0) == 24, "Wrong offset for PxJacobianRow.angular0, expected 24 got %v", offset_of(PxJacobianRow, angular0))
    testing.expectf(t, offset_of(PxJacobianRow, angular1) == 36, "Wrong offset for PxJacobianRow.angular1, expected 36 got %v", offset_of(PxJacobianRow, angular1))
    testing.expectf(t, size_of(PxJacobianRow) == 48, "Wrong size for type PxJacobianRow, expected 48 got %v", size_of(PxJacobianRow))
}

@(test)
test_layout_PxContactJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxContactJoint) == 24, "Wrong size for type PxContactJoint, expected 24 got %v", size_of(PxContactJoint))
}

@(test)
test_layout_PxFixedJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxFixedJoint) == 24, "Wrong size for type PxFixedJoint, expected 24 got %v", size_of(PxFixedJoint))
}

@(test)
test_layout_PxJointLimitParameters :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJointLimitParameters, bounceThreshold) == 4, "Wrong offset for PxJointLimitParameters.bounceThreshold, expected 4 got %v", offset_of(PxJointLimitParameters, bounceThreshold))
    testing.expectf(t, offset_of(PxJointLimitParameters, stiffness) == 8, "Wrong offset for PxJointLimitParameters.stiffness, expected 8 got %v", offset_of(PxJointLimitParameters, stiffness))
    testing.expectf(t, offset_of(PxJointLimitParameters, damping) == 12, "Wrong offset for PxJointLimitParameters.damping, expected 12 got %v", offset_of(PxJointLimitParameters, damping))
    testing.expectf(t, offset_of(PxJointLimitParameters, contactDistance_deprecated) == 16, "Wrong offset for PxJointLimitParameters.contactDistance_deprecated, expected 16 got %v", offset_of(PxJointLimitParameters, contactDistance_deprecated))
    testing.expectf(t, size_of(PxJointLimitParameters) == 20, "Wrong size for type PxJointLimitParameters, expected 20 got %v", size_of(PxJointLimitParameters))
}

@(test)
test_layout_PxJointLinearLimit :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJointLinearLimit, value) == 20, "Wrong offset for PxJointLinearLimit.value, expected 20 got %v", offset_of(PxJointLinearLimit, value))
    testing.expectf(t, size_of(PxJointLinearLimit) == 24, "Wrong size for type PxJointLinearLimit, expected 24 got %v", size_of(PxJointLinearLimit))
}

@(test)
test_layout_PxJointLinearLimitPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJointLinearLimitPair, upper) == 20, "Wrong offset for PxJointLinearLimitPair.upper, expected 20 got %v", offset_of(PxJointLinearLimitPair, upper))
    testing.expectf(t, offset_of(PxJointLinearLimitPair, lower) == 24, "Wrong offset for PxJointLinearLimitPair.lower, expected 24 got %v", offset_of(PxJointLinearLimitPair, lower))
    testing.expectf(t, size_of(PxJointLinearLimitPair) == 28, "Wrong size for type PxJointLinearLimitPair, expected 28 got %v", size_of(PxJointLinearLimitPair))
}

@(test)
test_layout_PxJointAngularLimitPair :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJointAngularLimitPair, upper) == 20, "Wrong offset for PxJointAngularLimitPair.upper, expected 20 got %v", offset_of(PxJointAngularLimitPair, upper))
    testing.expectf(t, offset_of(PxJointAngularLimitPair, lower) == 24, "Wrong offset for PxJointAngularLimitPair.lower, expected 24 got %v", offset_of(PxJointAngularLimitPair, lower))
    testing.expectf(t, size_of(PxJointAngularLimitPair) == 28, "Wrong size for type PxJointAngularLimitPair, expected 28 got %v", size_of(PxJointAngularLimitPair))
}

@(test)
test_layout_PxJointLimitCone :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJointLimitCone, yAngle) == 20, "Wrong offset for PxJointLimitCone.yAngle, expected 20 got %v", offset_of(PxJointLimitCone, yAngle))
    testing.expectf(t, offset_of(PxJointLimitCone, zAngle) == 24, "Wrong offset for PxJointLimitCone.zAngle, expected 24 got %v", offset_of(PxJointLimitCone, zAngle))
    testing.expectf(t, size_of(PxJointLimitCone) == 28, "Wrong size for type PxJointLimitCone, expected 28 got %v", size_of(PxJointLimitCone))
}

@(test)
test_layout_PxJointLimitPyramid :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxJointLimitPyramid, yAngleMin) == 20, "Wrong offset for PxJointLimitPyramid.yAngleMin, expected 20 got %v", offset_of(PxJointLimitPyramid, yAngleMin))
    testing.expectf(t, offset_of(PxJointLimitPyramid, yAngleMax) == 24, "Wrong offset for PxJointLimitPyramid.yAngleMax, expected 24 got %v", offset_of(PxJointLimitPyramid, yAngleMax))
    testing.expectf(t, offset_of(PxJointLimitPyramid, zAngleMin) == 28, "Wrong offset for PxJointLimitPyramid.zAngleMin, expected 28 got %v", offset_of(PxJointLimitPyramid, zAngleMin))
    testing.expectf(t, offset_of(PxJointLimitPyramid, zAngleMax) == 32, "Wrong offset for PxJointLimitPyramid.zAngleMax, expected 32 got %v", offset_of(PxJointLimitPyramid, zAngleMax))
    testing.expectf(t, size_of(PxJointLimitPyramid) == 36, "Wrong size for type PxJointLimitPyramid, expected 36 got %v", size_of(PxJointLimitPyramid))
}

@(test)
test_layout_PxPrismaticJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPrismaticJoint) == 24, "Wrong size for type PxPrismaticJoint, expected 24 got %v", size_of(PxPrismaticJoint))
}

@(test)
test_layout_PxRevoluteJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRevoluteJoint) == 24, "Wrong size for type PxRevoluteJoint, expected 24 got %v", size_of(PxRevoluteJoint))
}

@(test)
test_layout_PxSphericalJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxSphericalJoint) == 24, "Wrong size for type PxSphericalJoint, expected 24 got %v", size_of(PxSphericalJoint))
}

@(test)
test_layout_PxD6JointDrive :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxD6JointDrive, forceLimit) == 8, "Wrong offset for PxD6JointDrive.forceLimit, expected 8 got %v", offset_of(PxD6JointDrive, forceLimit))
    testing.expectf(t, offset_of(PxD6JointDrive, flags) == 12, "Wrong offset for PxD6JointDrive.flags, expected 12 got %v", offset_of(PxD6JointDrive, flags))
    testing.expectf(t, size_of(PxD6JointDrive) == 16, "Wrong size for type PxD6JointDrive, expected 16 got %v", size_of(PxD6JointDrive))
}

@(test)
test_layout_PxD6Joint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxD6Joint) == 24, "Wrong size for type PxD6Joint, expected 24 got %v", size_of(PxD6Joint))
}

@(test)
test_layout_PxGearJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxGearJoint) == 24, "Wrong size for type PxGearJoint, expected 24 got %v", size_of(PxGearJoint))
}

@(test)
test_layout_PxRackAndPinionJoint :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRackAndPinionJoint) == 24, "Wrong size for type PxRackAndPinionJoint, expected 24 got %v", size_of(PxRackAndPinionJoint))
}

@(test)
test_layout_PxGroupsMask :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxGroupsMask, bits1) == 2, "Wrong offset for PxGroupsMask.bits1, expected 2 got %v", offset_of(PxGroupsMask, bits1))
    testing.expectf(t, offset_of(PxGroupsMask, bits2) == 4, "Wrong offset for PxGroupsMask.bits2, expected 4 got %v", offset_of(PxGroupsMask, bits2))
    testing.expectf(t, offset_of(PxGroupsMask, bits3) == 6, "Wrong offset for PxGroupsMask.bits3, expected 6 got %v", offset_of(PxGroupsMask, bits3))
    testing.expectf(t, size_of(PxGroupsMask) == 8, "Wrong size for type PxGroupsMask, expected 8 got %v", size_of(PxGroupsMask))
}

@(test)
test_layout_PxDefaultErrorCallback :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultErrorCallback) == 8, "Wrong size for type PxDefaultErrorCallback, expected 8 got %v", size_of(PxDefaultErrorCallback))
}


@(test)
test_layout_PxMassProperties :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxMassProperties, centerOfMass) == 36, "Wrong offset for PxMassProperties.centerOfMass, expected 36 got %v", offset_of(PxMassProperties, centerOfMass))
    testing.expectf(t, offset_of(PxMassProperties, mass) == 48, "Wrong offset for PxMassProperties.mass, expected 48 got %v", offset_of(PxMassProperties, mass))
    testing.expectf(t, size_of(PxMassProperties) == 52, "Wrong size for type PxMassProperties, expected 52 got %v", size_of(PxMassProperties))
}



@(test)
test_layout_PxMeshOverlapUtil :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxMeshOverlapUtil) == 1040, "Wrong size for type PxMeshOverlapUtil, expected 1040 got %v", size_of(PxMeshOverlapUtil))
}

@(test)
test_layout_PxXmlMiscParameter :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxXmlMiscParameter, scale) == 12, "Wrong offset for PxXmlMiscParameter.scale, expected 12 got %v", offset_of(PxXmlMiscParameter, scale))
    testing.expectf(t, size_of(PxXmlMiscParameter) == 20, "Wrong size for type PxXmlMiscParameter, expected 20 got %v", size_of(PxXmlMiscParameter))
}


@(test)
test_layout_PxDefaultCpuDispatcher :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxDefaultCpuDispatcher) == 8, "Wrong size for type PxDefaultCpuDispatcher, expected 8 got %v", size_of(PxDefaultCpuDispatcher))
}




@(test)
test_layout_PxBatchQueryExt :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxBatchQueryExt) == 8, "Wrong size for type PxBatchQueryExt, expected 8 got %v", size_of(PxBatchQueryExt))
}

@(test)
test_layout_PxCustomSceneQuerySystem :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCustomSceneQuerySystem) == 8, "Wrong size for type PxCustomSceneQuerySystem, expected 8 got %v", size_of(PxCustomSceneQuerySystem))
}

@(test)
test_layout_PxCustomSceneQuerySystemAdapter :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxCustomSceneQuerySystemAdapter) == 8, "Wrong size for type PxCustomSceneQuerySystemAdapter, expected 8 got %v", size_of(PxCustomSceneQuerySystemAdapter))
}


@(test)
test_layout_PxPoissonSampler :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPoissonSampler) == 8, "Wrong size for type PxPoissonSampler, expected 8 got %v", size_of(PxPoissonSampler))
}

@(test)
test_layout_PxTriangleMeshPoissonSampler :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxTriangleMeshPoissonSampler) == 8, "Wrong size for type PxTriangleMeshPoissonSampler, expected 8 got %v", size_of(PxTriangleMeshPoissonSampler))
}


@(test)
test_layout_PxRepXObject :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxRepXObject, serializable) == 8, "Wrong offset for PxRepXObject.serializable, expected 8 got %v", offset_of(PxRepXObject, serializable))
    testing.expectf(t, offset_of(PxRepXObject, id) == 16, "Wrong offset for PxRepXObject.id, expected 16 got %v", offset_of(PxRepXObject, id))
    testing.expectf(t, size_of(PxRepXObject) == 24, "Wrong size for type PxRepXObject, expected 24 got %v", size_of(PxRepXObject))
}

@(test)
test_layout_PxRepXInstantiationArgs :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, offset_of(PxRepXInstantiationArgs, cooker) == 8, "Wrong offset for PxRepXInstantiationArgs.cooker, expected 8 got %v", offset_of(PxRepXInstantiationArgs, cooker))
    testing.expectf(t, offset_of(PxRepXInstantiationArgs, stringTable) == 16, "Wrong offset for PxRepXInstantiationArgs.stringTable, expected 16 got %v", offset_of(PxRepXInstantiationArgs, stringTable))
    testing.expectf(t, size_of(PxRepXInstantiationArgs) == 24, "Wrong size for type PxRepXInstantiationArgs, expected 24 got %v", size_of(PxRepXInstantiationArgs))
}

@(test)
test_layout_PxRepXSerializer :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxRepXSerializer) == 8, "Wrong size for type PxRepXSerializer, expected 8 got %v", size_of(PxRepXSerializer))
}

@(test)
test_layout_PxPvd :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPvd) == 8, "Wrong size for type PxPvd, expected 8 got %v", size_of(PxPvd))
}

@(test)
test_layout_PxPvdTransport :: proc(t: ^testing.T) {
    using physx
    testing.expectf(t, size_of(PxPvdTransport) == 8, "Wrong size for type PxPvdTransport, expected 8 got %v", size_of(PxPvdTransport))
}
