#include <RadeonProRender.h>

#include <MaterialXCore/Document.h>
#include <MaterialXFormat/File.h>

class HsMtlxLoader {
public:
    HsMtlxLoader() = default;

    void SetupStdlib(MaterialX::FilePathVec const& libraryNames, MaterialX::FileSearchPath const& searchPath);
    void SetLogging(bool enable) { _loggingEnabled = enable; }

    enum OutputType {
        Surface,
        Displacement,
        kMaxNumOutputs
    };

    struct Result {
        /// All rpr nodes that form a material graph
        rpr_material_node* nodes = nullptr;

        /// Number of elements in nodes array
        size_t numNodes;

        size_t rootNodeIndices[kMaxNumOutputs];
        static const size_t kInvalidRootNodeIndex = size_t(-1);

        struct ImageNode {
            /// Path to the image file
            /// It's responsibility of the loader user to setup rpr_image and bind it to rprNode
            std::string filepath;

            /// Image texture node
            rpr_material_node rprNode;
        };

        ImageNode* imageNodes = nullptr;
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
