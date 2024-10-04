set -e
(cd ../pxbind/ && cargo run)
clang++ -DNDEBUG structgen.cpp -I../physx/physx/include/ -o structgen
./structgen
clang-17 structgen_out.h --std=c2x
