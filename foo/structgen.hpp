#include <cassert>
#include <cstdio>
#include <cstdint>
#include <vector>

struct RustCheck {
    const char* rname;
    uint32_t size;
};

struct PodStructGen {
    PodStructGen() {
        cfile = fopen("structgen_out.hpp", "w");
        rfile = fopen("structgen_out.h", "w");

		fputs( "#include <cstdint>\n", cfile);
                fputs("#include <stdint.h>\n", rfile);
		fputs("#include \"physx_generated_enums.h\"\n", rfile);

		fputs("#define FORWARD_DECL_STRUCT(type) typedef struct physx_##type physx_##type;\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxProfilerCallback);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxTriangleMesh);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxHeightField);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxBroadPhaseCallback);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxContactModifyCallback);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxCpuDispatcher);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxCCDContactModifyCallback);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxSimulationEventCallback);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxTetrahedronMesh);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxCustomGeometryCallbacks);\n", rfile);
		fputs("FORWARD_DECL_STRUCT(PxSimulationFilterCallback);\n", rfile);
                fputs("FORWARD_DECL_STRUCT(PxSceneQuerySystem);\n", rfile);
                fputs("FORWARD_DECL_STRUCT(PxContactPair);\n", rfile);
                fputs("FORWARD_DECL_STRUCT(PxController);\n", rfile);
                fputs("FORWARD_DECL_STRUCT(PxQueryFilterCallback);\n", rfile);
                fputs("FORWARD_DECL_STRUCT(PxControllerFilterCallback);\n",
                      rfile);
                fputs("FORWARD_DECL_STRUCT(PxControllerBehaviorCallback);\n",
                      rfile);
                fputs("FORWARD_DECL_STRUCT(PxUserControllerHitReport);\n",
                      rfile);
                fputs("FORWARD_DECL_STRUCT(PxCooking);\n", rfile);
                fputs("FORWARD_DECL_STRUCT(PxStringTable);\n",
                      rfile);
//		fputs( "#include \"physx_generated.h\"\n", rfile);
			  }

    void finish() {
        fclose(cfile);

        for (const auto& rc : rust_checks) {
            fprintf(
                rfile,
                "        static_assert(sizeof(%s) == %u, \"assertion failed:\");\n",
                rc.rname,
                rc.size
            );
        }
        fclose(rfile);
    }

    void pass_thru(const char *code) {
        fputs(code, cfile);
		//        fputs(code, rfile);
	}

    void begin_struct(const char* cname, const char* rname) {
        fprintf(cfile, "struct %s {\n", cname);


        fprintf(rfile, "typedef struct %s {\n", rname);

        this->rname = rname;
        pos = 0;
        padIdx = 0;
    }

    void emit_padding(uint32_t bytes) {
        fprintf(cfile, "    char structgen_pad%u[%u];\n", padIdx, bytes);
        fprintf(rfile, "    char structgen_pad%u[%u];\n", padIdx, bytes);
        ++padIdx;
    }

    void add_field(
        const char* cppDecl,
        const char* rustName,
        const char* rustType,
        size_t size,
        size_t offset) {
        assert(offset >= pos);
        if (offset > pos) {
            emit_padding(uint32_t(offset - pos));
            pos = offset;
        }
        fprintf(cfile, "    %s;\n", cppDecl);
        fprintf(rfile, "    %s %s;\n", rustType, rustName);
        pos += size;
    }

    void end_struct(size_t size) {
        assert(size >= pos);
        if (size > pos) {
            emit_padding(uint32_t(size - pos));
        }
        fputs("};\n", cfile);
        fprintf(rfile, "} %s;\n", this->rname);

        rust_checks.emplace_back(RustCheck { rname, uint32_t(size) });
    }

  private:
    std::vector<RustCheck> rust_checks;
    FILE* cfile;
    FILE* rfile;
    const char* rname;
    size_t pos;
    uint32_t padIdx;
			  };
