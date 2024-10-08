prepare:
    cd codegen/pxbind/ && cargo run -- --stage 0

structgen: prepare
    cd codegen && clang++ -DNDEBUG -g structgen.cpp -Iphysx/physx/include/ -o structgen && ./structgen

validate: structgen
    cd codegen && clang++ -DNDEBUG  -Iphysx/physx/include/ src/physx_api.cpp structgen_out.hpp

bindgen: structgen
    cd codegen/pxbind && cargo run -- --stage 1

build:  bindgen
    cd codegen && cargo build
    cp codegen/*.so .

gen:
    cd codegen && clang++ -DNDEBUG -g structgen.cpp -Iphysx/physx/include/ -o structgen && ./structgen
    cd codegen/pxbind && cargo run -- --stage 1

gentest: gen
    odin test tests
