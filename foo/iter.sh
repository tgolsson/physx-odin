set -e
echo "building rust generator"
(cd ../pxbind/ && cargo run)

echo "building cpp -> target lang generator"
clang++ -DNDEBUG structgen.cpp -I../physx/physx/include/ -o structgen

echo "running target lang generator"
./structgen

echo "building target lang header"
clang-17 structgen_out.h --std=c2x
