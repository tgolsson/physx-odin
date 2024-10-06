set -e
#echo "building rust generator"
#(cd ../pxbind/ && cargo run -- --stage 0)

echo "building cpp -> target lang generator"
clang++ -DNDEBUG structgen.cpp -I../physx/physx/include/ -o structgen


echo "generating size metadata"
./structgen

echo "generating target lang"
(cd ../pxbind/ && cargo run -- --stage 1)

echo "building target lang header"
odin build .

# clang-14 structgen_out.h --std=c2x
