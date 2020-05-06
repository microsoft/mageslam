# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

set(LIBRARY_NAME "Arcana")

set(SOURCES
    "Shared/arcana/analysis/binary_iterator.h"
    "Shared/arcana/analysis/crc32.cpp"
    "Shared/arcana/analysis/data_point.h"
    "Shared/arcana/analysis/Determinator.cpp"
    "Shared/arcana/analysis/determinator.h"
    "Shared/arcana/analysis/introspector.h"
    "Shared/arcana/analysis/object_trace.cpp"
    "Shared/arcana/analysis/object_trace.h"
    "Shared/arcana/analysis/sha1.cpp"
    "Shared/arcana/analysis/sha1.h"
    "Shared/arcana/analysis/xray.cpp"
    "Shared/arcana/analysis/xray.h"
    "Shared/arcana/bob/bob_data.h"
    "Shared/arcana/bob/bob_entry_writer.h"
    "Shared/arcana/bob/bob_helpers.h"
    "Shared/arcana/bob/bob_reader.cpp"
    "Shared/arcana/bob/bob_reader.h"
    "Shared/arcana/bob/bob_stream_file.h"
    "Shared/arcana/bob/bob_writer.h"
    "Shared/arcana/bob/bob.h"
    "Shared/arcana/containers/index_pool.h"
    "Shared/arcana/containers/sorted_vector.h"
    "Shared/arcana/containers/ticketed_collection.h"
    "Shared/arcana/containers/unique_vector.h"
    "Shared/arcana/experimental/array.h"
    "Shared/arcana/functional/inplace_function.h"
    "Shared/arcana/messaging/mediator.h"
    "Shared/arcana/messaging/router.h"
    "Shared/arcana/scheduling/state_machine_state.h"
    "Shared/arcana/scheduling/state_machine.h"
    "Shared/arcana/streams/memory_device.h"
    "Shared/arcana/threading/internal/internal_task.h"
    "Shared/arcana/threading/affinity.h"
    "Shared/arcana/threading/blocking_concurrent_queue.h"
    "Shared/arcana/threading/cancellation.h"
    "Shared/arcana/threading/coroutine.h"
    "Shared/arcana/threading/dispatcher.h"
    "Shared/arcana/threading/fence.h"
    "Shared/arcana/threading/incrementing_counter.h"
    "Shared/arcana/threading/invoke_nth.h"
    "Shared/arcana/threading/nfence.h"
    "Shared/arcana/threading/parallel.h"
    "Shared/arcana/threading/pending_task_scope.h"
    "Shared/arcana/threading/semaphore.h"
    "Shared/arcana/threading/signal.cpp"
    "Shared/arcana/threading/signal.h"
    "Shared/arcana/threading/task.h"
    "Shared/arcana/utils/serialization/base_stream.h"
    "Shared/arcana/utils/serialization/custom_serialization.h"
    "Shared/arcana/utils/serialization/member_serializer.h"
    "Shared/arcana/utils/serialization/member.h"
    "Shared/arcana/utils/serialization/serializable.h"
    "Shared/arcana/utils/algorithm.h"
    "Shared/arcana/utils/functional.h"
    "Shared/arcana/utils/memory_extensions.h"
    "Shared/arcana/utils/stl_shim.h"
    "Shared/arcana/either.h"
    "Shared/arcana/errors.cpp"
    "Shared/arcana/errors.h"
    "Shared/arcana/exceptions.h"
    "Shared/arcana/expected.h"
    "Shared/arcana/finally_scope.h"
    "Shared/arcana/function_view.h"
    "Shared/arcana/iterators.h"
    "Shared/arcana/macros.h"
    "Shared/arcana/math.h"
    "Shared/arcana/mixin_ptr.h"
    "Shared/arcana/path.h"
    "Shared/arcana/propertybag.h"
    "Shared/arcana/reference_counter.h"
    "Shared/arcana/sentry.h"
    "Shared/arcana/string.h"
    "Shared/arcana/timer.h"
    "Shared/arcana/type_traits.h")

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME} PUBLIC "Shared")

target_link_libraries(${LIBRARY_NAME} PUBLIC
    BoostHeaders
    cereal
    GSL)

target_compile_definitions(${LIBRARY_NAME} PUBLIC "NOMINMMAX")

set_property(TARGET ${LIBRARY_NAME} PROPERTY FOLDER Dependencies)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
