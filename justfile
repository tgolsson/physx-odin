default:
    echo 'Hello, world!'

prepare:
    cd codegen/pxbind/ && cargo run -- --stage 0

structgen: prepare
    cd codegen && clang++ -DNDEBUG -g structgen.cpp -Iphysx/physx/include/ -o structgen && ./structgen

bindgen: structgen
    cd codegen/ && cargo run -- --stage 1
