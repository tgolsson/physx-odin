#include <cassert>
#include <cstdio>
#include <cstdint>
#include <vector>
#include <cerrno>

struct PodStructGen {
    PodStructGen() {
        cppfile = fopen("structgen_out.hpp", "w");
        definitions_file = fopen("structgen_out.json", "wb");
        fputs("#include <cstdint>\n", cppfile);
        fputs("#include <cstddef>\n", cppfile);
        fputs("{\n    \"structs\": [\n", definitions_file);
        current_indent = 8;
    }

    void finish() {
        fclose(cppfile);
		if (current_indent != 8) {
			current_indent = 8;
		}

		if (fieldsEmitted) {
			int res = fseek(definitions_file, -2, SEEK_CUR);

			if (res != 0) {
				printf("failed seek! %d - %d\n", errno, res);
			}

			fputs("\n", definitions_file);
		}


		current_indent -=4 ;
		indent();
        fputs("]\n", definitions_file);
		current_indent -=4 ;
		indent();
        fputs("}\n", definitions_file);
        fclose(definitions_file);
    }

    void pass_thru(const char* code) { fputs(code, cppfile); }

    void indent() {

        for (int i = 0; i < current_indent; ++i) {
			fputs(" ", definitions_file);
        }
    }

    void begin_struct(const char* cname, const char* truename) {
        fprintf(cppfile, "struct %s {\n", cname);
		indent();
        fputs("{\n", definitions_file);
		current_indent += 4;
        indent();
        fprintf(definitions_file, "\"name\": \"%s\",\n", truename);
		indent();
        fprintf(definitions_file, "\"fields\": [\n");
		current_indent += 4;
        pos = 0;
        padIdx = 0;
		fieldsEmitted = 0;
    }

    void emit_padding(uint32_t bytes, uint32_t offset, bool isBase = false) {
        fprintf(cppfile, "    char structgen_pad%u[%u];\n", padIdx, bytes);
		indent();
        if (!isBase) fprintf(definitions_file, "{\"name\": \"PAD\", \"size\": %u, \"offset\": %u},\n", bytes, offset);
		fieldsEmitted += 1;
        ++padIdx;
    }

    void add_field(const char *cppType, const char *cppName, size_t size,
                   size_t offset,  bool isBase
		) {
        assert(offset >= pos);
        if (offset > pos) {

            emit_padding(uint32_t(offset - pos), offset, isBase);
            pos = offset;
        }
        fprintf(cppfile, "    %s %s;\n", cppType, cppName);
		if (!isBase){
			indent();
			fprintf(definitions_file,
					"{\"name\": \"%s\", \"type\": \"%s\", "
					"\"offset\": %zu, \"size\": %zu},\n",
					cppName, cppType, offset, size);

			fieldsEmitted += 1;
                }
		previousWasBase = isBase;
		pos += size;

    }

    void add_reference(
        const char* cppType,
        const char* cppName,
        size_t size) {
		// fprintf(cppfile, "    %s %s;\n", cppType, cppName);
		// indent();
		// fprintf(definitions_file, "{\"name\": \"%s\", \"type\": \"%s\", \"offset\": 0, \"size\": %zu},\n", cppName, cppType, size);
        //  pos += size;
		// fieldsEmitted += 1;
    }



    void add_private_field(
        const char* cppType,
        const char* cppName,
        size_t size) {
        // fprintf(cppfile, "    %s %s;\n", cppType, cppName);
		// indent();
        //  fprintf(definitions_file, "{\"name\": \"%s\", \"type\": \"%s\", \"offset\": 0, \"size\": %zu},\n", cppName, cppType, size);
		//  pos += size;
		// fieldsEmitted += 1;
    }

    void end_struct(size_t size) {
        assert(size >= pos);
        if (size > pos) {
            emit_padding(uint32_t(size - pos), pos);
        }
		if (fieldsEmitted) {
			int res = fseek(definitions_file, -2, SEEK_CUR);

			if (res != 0) {
				printf("failed seek! %d - %d\n", errno, res);
			}

			fputs("\n", definitions_file);
		}

        fputs("};\n", cppfile);
		current_indent -=4 ;
		indent();
        fputs("],\n", definitions_file);
		indent();
		fprintf(definitions_file, "\"size\": %u\n", uint32_t(size));
		current_indent -= 4;
		indent();
        fputs("},\n", definitions_file);
    }

  private:
    FILE* cppfile;
    FILE* definitions_file;
    size_t pos;
    uint32_t padIdx;
	size_t current_indent;
        size_t fieldsEmitted;
	bool previousWasBase;
};
