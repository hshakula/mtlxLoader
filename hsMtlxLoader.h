#include <RadeonProRender.h>

#include <MaterialXCore/Document.h>
#include <MaterialXFormat/File.h>

class HsMtlxLoader {
public:
    HsMtlxLoader() = default;

    void SetupStdlib(MaterialX::FilePathVec const& libraryNames, MaterialX::FileSearchPath const& searchPath);
    void SetLogging(bool enable) { _loggingEnabled = enable; }

    struct Result {
        /// All rpr nodes that form a material graph
        rpr_material_node* nodes;

        /// Number of elements in nodes array
        size_t numNodes;

        /// Index of the root node of the surface graph
        size_t surfaceRootNodeIdx;

        struct ImageNode {
            /// Path to the image file
            /// It's responsibility of the loader user to setup rpr_image and bind it to rprNode
            std::string filepath;

            /// Image texture node
            rpr_material_node rprNode;
        };

        ImageNode* imageNodes;
        size_t numImageNodes;
    };

    /// Parses provided \p mtlxDocument, 
    Result Load(MaterialX::Document* mtlxDocument, rpr_material_system rprMatSys);

    /// Reference function on how properly to release HsMtlxLoader::Result
    static void Release(Result* result) {
        if (!result || !result->nodes) {
            return;
        }

        for (size_t i = 0; i < result->numNodes; ++i) {
            if (result->nodes[i]) {
                rprObjectDelete(result->nodes[i]);
            }
        }
        delete[] result->nodes;
        if (result->imageNodes) {
            delete[] result->imageNodes;
        }

        std::memset(result, 0, sizeof(*result));
    }

private:
    MaterialX::DocumentPtr _stdlib;
    bool _loggingEnabled = false;
};
