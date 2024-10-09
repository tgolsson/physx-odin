prepare:
    cd codegen/pxbind/ && cargo run -- --stage 0

structgen: prepare
    cd codegen && clang++ -DNDEBUG -g structgen.cpp -Iphysx/physx/include/ -o structgen && ./structgen

validate: structgen
    cd codegen && clang++ -DNDEBUG  -Iphysx/physx/include/ src/physx_api.cpp structgen_out.hpp

bindgen: structgen
    cd codegen/pxbind && cargo run -- --stage 1

build:  bindgen
    cd codegen && cargo run && cargo run --release

gen:
    cd codegen && clang++ -DNDEBUG -g structgen.cpp -Iphysx/physx/include/ -o structgen && ./structgen
    cd codegen/pxbind && cargo run -- --stage 1

test:
    odin test tests

gentest: gen test
    echo "DONE"
