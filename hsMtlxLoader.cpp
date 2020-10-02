#include "hsMtlxLoader.h"

#include <MaterialXFormat/Util.h> // mx::loadLibraries

#include <map>
#include <cstdarg>

namespace mx = MaterialX;

namespace {

//------------------------------------------------------------------------------
// Direct mappings of standard mtlx nodes to RPR nodes
//------------------------------------------------------------------------------

struct Mtlx2Rpr {
    struct Node {
        rpr_material_node_type id;
        std::map<std::string, rpr_material_node_input> inputs;
        using InputsValueType = decltype(inputs)::value_type;

        Node() = default;
        Node(rpr_material_node_type id, std::initializer_list<InputsValueType>&& inputs)
            : id(id), inputs(std::move(inputs)) {
        }
    };

    std::map<std::string, Node> nodes;
    std::map<std::string, rpr_material_node_arithmetic_operation> arithmeticOps;

    Mtlx2Rpr() {
        nodes["diffuse_brdf"] = {
            RPR_MATERIAL_NODE_MATX_DIFFUSE_BRDF, {
                {"color", RPR_MATERIAL_INPUT_COLOR},
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
            }
        };
        nodes["dielectric_brdf"] = {
            RPR_MATERIAL_NODE_MATX_DIELECTRIC_BRDF, {
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"tint", RPR_MATERIAL_INPUT_TINT},
                {"ior", RPR_MATERIAL_INPUT_IOR},
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
                {"tangent", RPR_MATERIAL_INPUT_TANGENT},
                {"distribution", RPR_MATERIAL_INPUT_DISTRIBUTION},
                {"base", RPR_MATERIAL_INPUT_BASE},
            }
        };
        nodes["generalized_schlick_brdf"] = {
            RPR_MATERIAL_NODE_MATX_GENERALIZED_SCHLICK_BRDF, {
                {"color0", RPR_MATERIAL_INPUT_COLOR0},
                {"color90", RPR_MATERIAL_INPUT_COLOR1},
                {"exponent", RPR_MATERIAL_INPUT_EXPONENT},
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
                {"tangent", RPR_MATERIAL_INPUT_TANGENT},
                {"distribution", RPR_MATERIAL_INPUT_DISTRIBUTION},
                {"base", RPR_MATERIAL_INPUT_BASE},
            }
        };
        nodes["dielectric_btdf"] = {
            RPR_MATERIAL_NODE_MATX_DIELECTRIC_BTDF, {
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"tint", RPR_MATERIAL_INPUT_COLOR},
                {"ior", RPR_MATERIAL_INPUT_IOR},
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
                {"tangent", RPR_MATERIAL_INPUT_TANGENT},
                {"distribution", RPR_MATERIAL_INPUT_DISTRIBUTION},
                {"interior", RPR_MATERIAL_INPUT_INTERIOR},
            }
        };
        nodes["sheen_brdf"] = {
            RPR_MATERIAL_NODE_MATX_SHEEN_BRDF, {
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"color", RPR_MATERIAL_INPUT_COLOR},
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
                {"base", RPR_MATERIAL_INPUT_BASE},
            }
        };
        nodes["subsurface_brdf"] = {
            RPR_MATERIAL_NODE_MATX_SUBSURFACE_BRDF, {
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"color", RPR_MATERIAL_INPUT_COLOR},
                {"radius", RPR_MATERIAL_INPUT_RADIUS},
                {"anisotropy", RPR_MATERIAL_INPUT_ANISOTROPIC},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
            }
        };
        nodes["diffuse_btdf"] = {
            RPR_MATERIAL_NODE_MATX_DIFFUSE_BTDF, {
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"color", RPR_MATERIAL_INPUT_COLOR},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
            }
        };
        nodes["conductor_brdf"] = {
            RPR_MATERIAL_NODE_MATX_CONDUCTOR_BRDF, {
                {"weight", RPR_MATERIAL_INPUT_WEIGHT},
                {"reflectivity", RPR_MATERIAL_INPUT_REFLECTIVITY},
                {"edge_color", RPR_MATERIAL_INPUT_EDGE_COLOR},
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
                {"tangent", RPR_MATERIAL_INPUT_TANGENT},
                {"distribution", RPR_MATERIAL_INPUT_DISTRIBUTION},
            }
        };
        nodes["fresnel"] = {
            RPR_MATERIAL_NODE_MATX_FRESNEL, {
                {"ior", RPR_MATERIAL_INPUT_IOR},
                {"normal", RPR_MATERIAL_INPUT_NORMAL},
                {"viewdirection", RPR_MATERIAL_INPUT_VIEW_DIRECTION},
            }
        };
        nodes["constant"] = {
            RPR_MATERIAL_NODE_CONSTANT_TEXTURE, {
                {"value", RPR_MATERIAL_INPUT_VALUE},
            }
        };
        nodes["mix"] = {
            RPR_MATERIAL_NODE_BLEND_VALUE, {
                {"fg", RPR_MATERIAL_INPUT_COLOR1},
                {"bg", RPR_MATERIAL_INPUT_COLOR0},
                {"mix", RPR_MATERIAL_INPUT_WEIGHT},
            }
        };
        nodes["ifgreater"] = {
            RPR_MATERIAL_NODE_MATX_IFGREATER, {
                {"value1", RPR_MATERIAL_INPUT_0},
                {"value2", RPR_MATERIAL_INPUT_1},
                {"in1", RPR_MATERIAL_INPUT_COLOR0},
                {"in2", RPR_MATERIAL_INPUT_COLOR1},
            }
        };
        nodes["normalize"] = {
            RPR_MATERIAL_NODE_MATX_NORMALIZE, {
                {"in", RPR_MATERIAL_INPUT_COLOR},
            }
        };
        nodes["luminance"] = {
            RPR_MATERIAL_NODE_MATX_LUMINANCE, {
                {"in", RPR_MATERIAL_INPUT_0},
                {"lumacoeffs", RPR_MATERIAL_INPUT_LUMACOEFF},
            }
        };
        nodes["convert"] = {
            RPR_MATERIAL_NODE_MATX_CONVERT, {
                {"in", RPR_MATERIAL_INPUT_0},
            }
        };
        nodes["rotate3d"] = {
            RPR_MATERIAL_NODE_MATX_ROTATE3D, {
                {"in", RPR_MATERIAL_INPUT_0},
                {"amount", RPR_MATERIAL_INPUT_AMOUNT},
                {"axis", RPR_MATERIAL_INPUT_AXIS},
            }
        };
        nodes["roughness_anisotropy"] = {
            RPR_MATERIAL_NODE_MATX_ROUGHNESS_ANISOTROPY, {
                {"roughness", RPR_MATERIAL_INPUT_ROUGHNESS},
                {"anisotropy", RPR_MATERIAL_INPUT_ANISOTROPIC},
            }
        };
        nodes["noise3d"] = {
            RPR_MATERIAL_NODE_MATX_NOISE3D, {
                {"amplitude", RPR_MATERIAL_INPUT_AMPLITUDE},
                {"pivot", RPR_MATERIAL_INPUT_PIVOT},
                {"position", RPR_MATERIAL_INPUT_POSITION},
            }
        };
        nodes["normalmap"] = {
            RPR_MATERIAL_NODE_NORMAL_MAP, {
                {"in", RPR_MATERIAL_INPUT_COLOR},
                {"scale", RPR_MATERIAL_INPUT_SCALE},
            }
        };
        nodes["normalize"] = {
            RPR_MATERIAL_NODE_MATX_NORMALIZE, {
                {"in", RPR_MATERIAL_INPUT_COLOR},
            }
        };
        nodes["position"] = {RPR_MATERIAL_NODE_MATX_POSITION, {}};

        auto addArithmeticNode = [this](const char* name, rpr_material_node_arithmetic_operation op, int numArgs) {
            auto& mapping = nodes[name];
            mapping.id = RPR_MATERIAL_NODE_ARITHMETIC;

            arithmeticOps[name] = op;

            if (numArgs == 1) {
                mapping.inputs["in"] = RPR_MATERIAL_INPUT_COLOR0;
            } else {
                mapping.inputs["in1"] = RPR_MATERIAL_INPUT_COLOR0;
                mapping.inputs["in2"] = RPR_MATERIAL_INPUT_COLOR1;

                if (numArgs > 2) mapping.inputs["in3"] = RPR_MATERIAL_INPUT_COLOR2;
                if (numArgs > 3) mapping.inputs["in4"] = RPR_MATERIAL_INPUT_COLOR3;
            }
        };

        addArithmeticNode("sin", RPR_MATERIAL_NODE_OP_SIN, 1);
        addArithmeticNode("cos", RPR_MATERIAL_NODE_OP_COS, 1);
        addArithmeticNode("absval", RPR_MATERIAL_NODE_OP_ABS, 1);
        addArithmeticNode("floor", RPR_MATERIAL_NODE_OP_FLOOR, 1);
        addArithmeticNode("normalize", RPR_MATERIAL_NODE_OP_NORMALIZE3, 1);
        addArithmeticNode("power", RPR_MATERIAL_NODE_OP_POW, 2);
        addArithmeticNode("add", RPR_MATERIAL_NODE_OP_ADD, 2);
        addArithmeticNode("subtract", RPR_MATERIAL_NODE_OP_SUB, 2);
        addArithmeticNode("multiply", RPR_MATERIAL_NODE_OP_MUL, 2);
        addArithmeticNode("divide", RPR_MATERIAL_NODE_OP_DIV, 2);
        addArithmeticNode("min", RPR_MATERIAL_NODE_OP_MIN, 2);
        addArithmeticNode("max", RPR_MATERIAL_NODE_OP_MAX, 2);
        addArithmeticNode("dotproduct", RPR_MATERIAL_NODE_OP_DOT3, 2);
        addArithmeticNode("modulo", RPR_MATERIAL_NODE_OP_MOD, 2);

        arithmeticOps["invert"] = RPR_MATERIAL_NODE_OP_SUB;
        nodes["invert"] = {
            RPR_MATERIAL_NODE_ARITHMETIC, {
                {"amount", RPR_MATERIAL_INPUT_COLOR0},
                {"in", RPR_MATERIAL_INPUT_COLOR1},
            }
        };

        // TODO: add custom implementations
        arithmeticOps["clamp"] = RPR_MATERIAL_NODE_OP_MAX;
        nodes["clamp"] = {
            RPR_MATERIAL_NODE_ARITHMETIC, {
                {"in", RPR_MATERIAL_INPUT_COLOR0},
                {"low", RPR_MATERIAL_INPUT_COLOR1},
            }
        };
    }
};

Mtlx2Rpr const& GetMtlx2Rpr() {
    static Mtlx2Rpr s_mtlx2rpr;
    return s_mtlx2rpr;
}

//------------------------------------------------------------------------------
// Loader context declarations
//------------------------------------------------------------------------------

struct Node;

enum LogScope {
    LSGlobal = -1,
    LSGraph,
    LSNested = LSGraph,
    LSNode,
    LSInput,
    LogScopeMax
};

struct LoaderContext {
    mx::Node* nodeRef;
    mx::ShaderRef* shaderRef;
    mx::Document* mtlxDocument;
    rpr_material_system rprMatSys;

    std::vector<HsMtlxLoader::Result::ImageNode> imageNodes;

    std::map<std::string, std::unique_ptr<Node>> globalNodes;
    Node* GetGlobalNode(mx::Node* mtlxNode);

    static const int kGlobalLogDepth = -1;
    int logDepth = kGlobalLogDepth;
    LogScope logScope = LSGlobal;
    bool logEnabled = true;

    void LogLine(const char* fmt, ...) {
        if (logEnabled) {
            if (logScope != LSGlobal) {
                int padding = 0;
                if (logDepth > 0) {
                    padding += logDepth * LogScopeMax;
                }
                padding += logScope;

                if (padding > 0) {
                    printf("%*s", padding, "");
                }
                printf("- ");
            }

            va_list ap;
            va_start(ap, fmt);
            vprintf(fmt, ap);
            va_end(ap);
        }
    }

    struct LogScopeGuard {
        LoaderContext* ctx;
        int previousLogDepth;
        LogScope previousScope;

        LogScopeGuard(LoaderContext* ctx, LogScope scope)
            : ctx(ctx), previousLogDepth(ctx->logDepth), previousScope(ctx->logScope) {

            ctx->logScope = scope;
            if (scope == LSGlobal) {
                ctx->logDepth = kGlobalLogDepth;
            } else if (scope == LSNested) {
                ctx->logDepth++;
            }
        }
        ~LogScopeGuard() {
            ctx->logScope = previousScope;
            ctx->logDepth = previousLogDepth;
        }
    };
    LogScopeGuard EnterLogScope(LogScope scope) { return LogScopeGuard(this, scope); }

    struct NodeRefOverScope {
        LoaderContext* ctx;
        mx::Node* oldNodeRef;

        NodeRefOverScope(LoaderContext* ctx, mx::Node* tmpNodeRef)
            : ctx(ctx), oldNodeRef(ctx->nodeRef) {
            ctx->nodeRef = tmpNodeRef;
        }
        ~NodeRefOverScope() {
            ctx->nodeRef = oldNodeRef;
        }
    };
    NodeRefOverScope OverrideNodeRef(mx::Node* nodeRef) { return NodeRefOverScope(this, nodeRef); }

    template <typename F>
    void TraverseSubNodes(Node* node, F&& cb) {
        cb(node);
        if (auto mtlxGraphNode = node->AsA<MtlxNodeGraphNode>()) {
            for (auto& entry : mtlxGraphNode->subNodes) {
                TraverseSubNodes(entry.second.get(), cb);
            }
        }
    }

    template <typename F>
    void TraverseNodes(Node* node, F&& cb) {
        TraverseSubNodes(node, cb);
        for (auto& globalNode : globalNodes) {
            TraverseSubNodes(globalNode.second.get(), cb);
        }
    }
};

//------------------------------------------------------------------------------
// Node declarations
//------------------------------------------------------------------------------

struct Node {
    using Ptr = std::unique_ptr<Node>;

    virtual ~Node() = default;

    /// Connect the current (upstream) node to the \p downstreamNode
    virtual rpr_status Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) = 0;
    virtual rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) = 0;
    virtual rpr_status SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) = 0;

    template <typename T>
    T* AsA() {
        return dynamic_cast<T*>(this);
    }

    static Node::Ptr Create(mx::Node* mtlxNode, LoaderContext* context);
};

/// The node with rpr_material_node underlying type
///
struct RprNode : public Node {
    bool isOwningRprNode;
    rpr_material_node rprNode;

    /// \p retainNode controls whether RprNode owns \p node
    RprNode(rpr_material_node node, bool retainNode);
    ~RprNode() override;

    rpr_status Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) override;

    rpr_status SetInput(rpr_material_node_input inputId, std::string const& valueString, std::string const& valueType, LoaderContext* context);
};

struct RprMappedNode : public RprNode {
    Mtlx2Rpr::Node const* rprNodeMapping;

    /// RprMappedNode takes ownership over \p node
    RprMappedNode(rpr_material_node node, Mtlx2Rpr::Node const* nodeMapping);

    rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) override;
};

/// The node with mx::NodeGraph underlying type
///
struct MtlxNodeGraphNode : public Node {
    mx::GraphElementPtr mtlxGraph;
    std::map<std::string, Node::Ptr> subNodes;

    struct NoOutputsError : public std::exception {};

    /// Throws NoOutputsError exception if mtlxGraph has no outputs
    MtlxNodeGraphNode(mx::GraphElementPtr mtlxGraph, LoaderContext* context);
    ~MtlxNodeGraphNode() override = default;

    rpr_status Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) override;

private:
    struct PendingConnection {
        mx::NodePtr upstreamNode;
        mx::OutputPtr upstreamNodeOutput;

        mx::NodePtr downstreamNode;
        mx::InputPtr downstreamInput;
    };
    Node* GetSubNode(mx::NodePtr const& mtlxNode, std::vector<PendingConnection>* pendingConnections, LoaderContext* context);

    struct InterfaceSocket {
        mx::NodePtr subNode;
        mx::InputPtr input;
    };
    std::map<std::string, InterfaceSocket> _interfaceSockets;

    struct InterfaceSocketQuery {
        Node* node = nullptr;
        mx::Input* inputElement = nullptr;
    };
    InterfaceSocketQuery GetInterfaceSocket(mx::Element* inputElement);
};

//------------------------------------------------------------------------------
// Utilities
//------------------------------------------------------------------------------

std::string to_string(mx::Edge const& edge) {
    std::string ret = edge.getUpstreamElement()->getName() + " -> " + edge.getDownstreamElement()->getName();
    if (auto connectingElem = edge.getConnectingElement()) {
        ret += " [" + connectingElem->getName() + "]";
    }
    return ret;
}

template <typename T>
std::shared_ptr<T> GetFirst(mx::Element const* element) {
    for (auto& child : element->getChildren()) {
        if (auto object = child->asA<T>()) {
            return object;
        }
    }
    return nullptr;
}

mx::OutputPtr GetOutput(mx::InterfaceElement* interfaceElement, mx::PortElement* portElement, LoaderContext* context) {
    // If the interface element has a few outputs, the port element must specify a target output
    //
    if (interfaceElement->getType() == mx::MULTI_OUTPUT_TYPE_STRING) {
        auto& targetOutputName = portElement->getOutputString();
        if (!targetOutputName.empty()) {
            context->LogLine("Error: invalid port element structure: output should be specified when connecting to multioutput element - %s\n", portElement->asString().c_str());
            return nullptr;
        }

        auto output = interfaceElement->getOutput(targetOutputName);
        if (!output) {
            context->LogLine("Error: invalid connection: cannot determine output - %s\n", portElement->asString().c_str());
        }

        return output;
    }

    return GetFirst<mx::Output>(interfaceElement);
}

//------------------------------------------------------------------------------
// Loader context implementation
//------------------------------------------------------------------------------

Node* LoaderContext::GetGlobalNode(mx::Node* node) {
    auto it = globalNodes.find(node->getName());
    if (it == globalNodes.end()) {
        auto logScope = EnterLogScope(LSGlobal);
        if (auto newNode = Node::Create(node, this)) {
            it = globalNodes.emplace(node->getName(), std::move(newNode)).first;
        }
    }

    if (it == globalNodes.end()) {
        return nullptr;
    }
    return it->second.get();
}

//------------------------------------------------------------------------------
// Node implementation
//------------------------------------------------------------------------------

// TODO: add error handling
Node::Ptr Node::Create(mx::Node* mtlxNode, LoaderContext* context) {
    rpr_material_node rprNode = nullptr;
    Mtlx2Rpr::Node const* rprNodeMapping = nullptr;

    // Check for nodes with special handling first
    //
    if (mtlxNode->getCategory() == "surface") {
        auto surfaceDef = mtlxNode->getNodeDef();
        // The surface node has 3 inputs: bsdf, edf and opacity.
        // Right now we can not implement bsdf and edf blending and,
        // as a workaround, our surface node simply transfers bsdf node further along connections
        //
        struct SurfaceNode : public RprNode {
            SurfaceNode() : RprNode(nullptr, false) {}

            rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override {
                if (inputElement->getName() == "bsdf") {
                    rprNode = inputNode;
                    return RPR_SUCCESS;
                } else {
                    context->LogLine("Unsupported surface input: %s\n", inputElement->getName());
                    return RPR_ERROR_UNSUPPORTED;
                }
            }
        };

        return std::make_unique<SurfaceNode>();
    } if (mtlxNode->getCategory() == "displacement") {
        // The displacement node is passthrough node - it transfers input unaltered
        //
        struct DisplacementNode : public RprNode {
            DisplacementNode() : RprNode(nullptr, false) {}

            rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override {
                if (inputElement->getName() == "displacement") {
                    rprNode = inputNode;
                    isOwningRprNode = false;
                    return RPR_SUCCESS;
                } else {
                    context->LogLine("Unsupported displacement input: %s\n", inputElement->getName());
                    return RPR_ERROR_UNSUPPORTED;
                }
            }

            rpr_status SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) override {
                if (inputElement->getName() == "displacement") {
                    if (rprNode && isOwningRprNode) {
                        rprObjectDelete(rprNode);
                    }

                    rprNode = nullptr;
                    auto status = rprMaterialSystemCreateNode(context->rprMatSys, RPR_MATERIAL_NODE_CONSTANT_TEXTURE, &rprNode);
                    if (status == RPR_SUCCESS) {
                        status = RprNode::SetInput(RPR_MATERIAL_INPUT_VALUE, value, valueType, context);
                        if (status == RPR_SUCCESS) {
                            isOwningRprNode = true;
                        } else {
                            rprObjectDelete(rprNode);
                            rprNode = nullptr;
                        }
                    }
                    return status;
                } else {
                    context->LogLine("Unsupported displacement input: %s\n", inputElement->getName());
                    return RPR_ERROR_UNSUPPORTED;
                }
            }
        };

        return std::make_unique<DisplacementNode>();
    } if (mtlxNode->getCategory() == "texcoord") {
        rprMaterialSystemCreateNode(context->rprMatSys, RPR_MATERIAL_NODE_INPUT_LOOKUP, &rprNode);
        rprMaterialNodeSetInputUByKey(rprNode, RPR_MATERIAL_INPUT_VALUE, RPR_MATERIAL_NODE_LOOKUP_UV);
    } else if (mtlxNode->getCategory() == "normal") {
        rprMaterialSystemCreateNode(context->rprMatSys, RPR_MATERIAL_NODE_INPUT_LOOKUP, &rprNode);
        rprMaterialNodeSetInputUByKey(rprNode, RPR_MATERIAL_INPUT_VALUE, RPR_MATERIAL_NODE_LOOKUP_N);
    } else if (mtlxNode->getCategory() == "image") {
        // TODO: add support of uaddressmode, vaddressmode, filtertype, default, layer
        // TODO: support "Image Filename Substitutions" (see spec)
        auto fileParam = mtlxNode->getParameter("file");
        if (!fileParam->getValueString().empty()) {
            rprMaterialSystemCreateNode(context->rprMatSys, RPR_MATERIAL_NODE_IMAGE_TEXTURE, &rprNode);

            static Mtlx2Rpr::Node s_imageMapping = {
                RPR_MATERIAL_NODE_IMAGE_TEXTURE, {
                    {"texcoord", RPR_MATERIAL_INPUT_UV}
                }
            };
            rprNodeMapping = &s_imageMapping;

            context->imageNodes.push_back({fileParam->getValueString(), rprNode});
        }
    } else if (mtlxNode->getCategory() == "swizzle") {
        // TODO: implement healthy man swizzle

        std::string channels;
        if (auto channelsParam = mtlxNode->getActiveParameter("channels")) {
            auto& valueString = channelsParam->getValueString();
            if (channelsParam->getType() == "string" &&
                !valueString.empty()) {
                channels = valueString;
            }
        }

        rpr_material_node_arithmetic_operation op;
        if (channels == "x") {
            op = RPR_MATERIAL_NODE_OP_SELECT_X;
        } else if (channels == "y") {
            op = RPR_MATERIAL_NODE_OP_SELECT_Y;
        } else {
            return nullptr;
        }

        rprMaterialSystemCreateNode(context->rprMatSys, RPR_MATERIAL_NODE_ARITHMETIC, &rprNode);
        rprMaterialNodeSetInputUByKey(rprNode, RPR_MATERIAL_INPUT_OP, op);

        static Mtlx2Rpr::Node s_swizzleMapping = {
            RPR_MATERIAL_NODE_ARITHMETIC, {
                {"in", RPR_MATERIAL_INPUT_COLOR0}
            }
        };
        rprNodeMapping = &s_swizzleMapping;

    // Then process standard nodes that can be mapped directly to RPR nodes
    //
    } else {
        // Some of the materialX standard nodes map to RPR differently depending on the node return type
        //
        if (mtlxNode->getCategory() == "mix" && mtlxNode->getType() == "BSDF") {
            // In RPR, mixing of two BSDFs can be done with RPR_MATERIAL_NODE_BLEND
            //
            static Mtlx2Rpr::Node bsdfMix = {
                RPR_MATERIAL_NODE_BLEND, {
                    {"fg", RPR_MATERIAL_INPUT_COLOR1},
                    {"bg", RPR_MATERIAL_INPUT_COLOR0},
                    {"mix", RPR_MATERIAL_INPUT_WEIGHT},
                }
            };
            rprNodeMapping = &bsdfMix;
        } else {
            auto rprNodeMappingIt = GetMtlx2Rpr().nodes.find(mtlxNode->getCategory());
            if (rprNodeMappingIt != GetMtlx2Rpr().nodes.end()) {
                rprNodeMapping = &rprNodeMappingIt->second;
            } else {
                // But direct mapping might not have been implemented yet or
                // this node might be of a custom definition
                //
                for (auto& nodeDef : context->mtlxDocument->getMatchingNodeDefs(mtlxNode->getCategory())) {
                    if (auto implementation = nodeDef->getImplementation()) {
                        if (auto nodeGraph = implementation->asA<mx::NodeGraph>()) {
                            auto nodeRefOver = context->OverrideNodeRef(mtlxNode);
                            return std::make_unique<MtlxNodeGraphNode>(std::move(nodeGraph), context);
                        } else {
                            // TODO: code generation required
                            break;
                        }
                    }
                }

                context->LogLine("Unsupported node: %s (%s)\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str());
            }
        }
    }

    if (!rprNode && rprNodeMapping) {
        auto status = rprMaterialSystemCreateNode(context->rprMatSys, rprNodeMapping->id, &rprNode);
        if (status != RPR_SUCCESS) {
            context->LogLine("Error: failed to create %s (%s) node: %d\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str(), status);
            return nullptr;
        }

        // For arithmetic nodes, we also must not forget to set operation
        //
        if (rprNodeMapping->id == RPR_MATERIAL_NODE_ARITHMETIC) {
            auto it = GetMtlx2Rpr().arithmeticOps.find(mtlxNode->getCategory());
            if (it != GetMtlx2Rpr().arithmeticOps.end()) {
                rprMaterialNodeSetInputUByKey(rprNode, RPR_MATERIAL_INPUT_OP, it->second);
            } else {
                context->LogLine("Error: unknown arithmetic node: %s (%s)", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str());
            }
        }
    }

    if (!rprNode) {
        return nullptr;
    }
    rprObjectSetName(rprNode, mtlxNode->getName().c_str());

    if (rprNodeMapping) {
        return std::make_unique<RprMappedNode>(rprNode, rprNodeMapping);
    } else {
        return std::make_unique<RprNode>(rprNode, true);
    }
}

//------------------------------------------------------------------------------
// MtlxNodeGraphNode implementation
//------------------------------------------------------------------------------

MtlxNodeGraphNode::MtlxNodeGraphNode(mx::GraphElementPtr graph, LoaderContext* context)
    : mtlxGraph(std::move(graph)) {

    context->LogLine("NodeGraph: %s\n", mtlxGraph->getName().c_str());
    auto graphLogScope = context->EnterLogScope(LSGraph);

    // To avoid recursion, we postpone the connection of nodes until the whole subgraph is built
    std::vector<PendingConnection> pendingConnections;

    for (auto& output : mtlxGraph->getOutputs()) {
        context->LogLine("Output: %s -> %s \n", output->getName().c_str(), output->getNodeName().c_str());

        PendingConnection entryConnection;

        // An output of a node graph must have a `nodename` attribute
        //
        entryConnection.downstreamNode = mtlxGraph->getNode(output->getNodeName());
        if (!entryConnection.downstreamNode) {
            context->LogLine("Error: invalid output %s\n", output->getName().c_str(), mtlxGraph->getName().c_str());
            continue;
        }

        pendingConnections.push_back(std::move(entryConnection));
    }

    if (pendingConnections.empty()) {
        throw NoOutputsError();
    }

    while (!pendingConnections.empty()) {
        auto connection = std::move(pendingConnections.back());
        pendingConnections.pop_back();

        Node* downstreamNode = GetSubNode(connection.downstreamNode, &pendingConnections, context);
        Node* upstreamNode = GetSubNode(connection.upstreamNode, &pendingConnections, context);

        if (downstreamNode && upstreamNode) {
            auto status = upstreamNode->Connect(connection.upstreamNodeOutput.get(), downstreamNode, connection.downstreamInput.get(), context);

            if (status == RPR_SUCCESS) {
                context->LogLine("Connected %s to %s\n", connection.upstreamNode->getName().c_str(), connection.downstreamNode->getName().c_str());
            }
        }
    }
}

Node* MtlxNodeGraphNode::GetSubNode(
    mx::NodePtr const& mtlxNode,
    std::vector<PendingConnection>* pendingConnections,
    LoaderContext* context) {
    if (!mtlxNode) {
        return nullptr;
    }

    auto subNodeIt = subNodes.find(mtlxNode->getName());
    if (subNodeIt != subNodes.end()) {
        return subNodeIt->second.get();
    }

    auto nodeHandle = Node::Create(mtlxNode.get(), context);
    if (!nodeHandle) {
        return nullptr;
    }

    auto node = nodeHandle.get();
    subNodes.emplace(mtlxNode->getName(), std::move(nodeHandle));

    context->LogLine("Node: %s (%s)\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str());
    auto nodeLogScope = context->EnterLogScope(LSNode);

    // The node may have zero or more inputs and parameter elements
    //
    for (auto& child : mtlxNode->getChildren()) {
        // ValueElement is a common base type
        //
        auto inputElement = child->asA<mx::ValueElement>();
        if (!inputElement) {
            context->LogLine("Error: %s %s: unexpected node child\n", child->getCategory().c_str(), child->getName().c_str());
            continue;
        }

        context->LogLine("%s %s\n", inputElement->getCategory().c_str(), inputElement->getName().c_str());
        auto inputLogScope = context->EnterLogScope(LSInput);

        // An element that provides a value for the current input
        //
        mx::ValueElementPtr valueElement;

        // If this input has an interface name then shaderRef or nodeRef might have overridden the default value
        //
        auto& interfaceName = inputElement->getInterfaceName();
        if (!interfaceName.empty()) {
            // Only Input can be an interface socket
            if (auto input = inputElement->asA<mx::Input>()) {
                InterfaceSocket interfaceSocket;
                interfaceSocket.input = std::move(input);
                interfaceSocket.subNode = mtlxNode;
                _interfaceSockets.emplace(interfaceName, interfaceSocket);
            }

            mx::ValueElementPtr interfaceValueElement;

            if (context->nodeRef) {
                interfaceValueElement = context->nodeRef->getInput(interfaceName);
            }

            if (!interfaceValueElement && context->shaderRef) {
                interfaceValueElement = context->shaderRef->getBindInput(interfaceName);
            }

            if (!interfaceValueElement) {
                // Otherwise, get the default value from a node definition
                //
                if (auto nodeGraph = mtlxGraph->asA<mx::NodeGraph>()) {
                    if (auto nodeDef = nodeGraph->getNodeDef()) {
                        interfaceValueElement = nodeDef->getValueElement(interfaceName);
                    }
                }
            }

            if (interfaceValueElement) {
                valueElement = interfaceValueElement;
            } else {
                context->LogLine("Error: interface name %s - no value\n", interfaceName.c_str());
                continue;
            }
        } else {
            valueElement = inputElement;
        }

        rpr_status status = RPR_SUCCESS;

        // A value element may be of three different types Input, BindInput, Parameter.
        // They all have their specific ways to specify an input value but also a common one -
        // `value` attribute that specifies a uniform value, this has the least precedence.

        // If a value element is of Input type, it can specify its value through:
        //  1) `nodename` attribute - an input connection to the node of the current graph
        //  2) `value` attribute - a uniform value
        //
        if (auto input = valueElement->asA<mx::Input>()) {
            auto& nodeName = input->getNodeName();
            if (!nodeName.empty()) {
                context->LogLine("nodename: %s\n", nodeName.c_str());

                auto mtlxUpstreamNode = mtlxGraph->getNode(nodeName);
                if (!mtlxUpstreamNode) {
                    context->LogLine("Error: no such node - %s\n", input->asString().c_str());
                    continue;
                }

                auto mtlxUpstreamNodeOutput = GetOutput(mtlxUpstreamNode.get(), input.get(), context);
                if (!mtlxUpstreamNodeOutput && mtlxUpstreamNode->getType() == mx::MULTI_OUTPUT_TYPE_STRING) {
                    continue;
                }

                // If upstream node is already created, connect output of upstream node to input of downstream node
                //
                auto upstreamNodeIt = subNodes.find(mtlxUpstreamNode->getName());
                if (upstreamNodeIt != subNodes.end()) {
                    status = upstreamNodeIt->second->Connect(mtlxUpstreamNodeOutput.get(), node, input.get(), context);
                } else {
                    // Otherwise, postpone the connection process until the upstream node is created
                    //
                    pendingConnections->emplace_back();
                    auto& pendingConnection = pendingConnections->back();
                    pendingConnection.downstreamNode = mtlxNode;
                    pendingConnection.downstreamInput = std::move(input);
                    pendingConnection.upstreamNode = std::move(mtlxUpstreamNode);
                    pendingConnection.upstreamNodeOutput = std::move(mtlxUpstreamNodeOutput);
                }

                if (status == RPR_SUCCESS) {
                    continue;
                }
            }

        // If a value element is of BindInput type, it can specify its value through:
        //  1) `output` attribute - the name of the nodegraph output
        //  2) `value` attribute - a uniform value
        //
        } else if (auto bindInput = valueElement->asA<mx::BindInput>()) {
            auto& outputName = bindInput->getOutputString();
            if (!outputName.empty()) {

                // The output attribute may point to the output of some node graph or free-standing (global) output
                //
                auto& nodeGraphName = bindInput->getNodeGraphString();
                if (!nodeGraphName.empty()) {
                    if (auto nodeGraph = context->mtlxDocument->getNodeGraph(nodeGraphName)) {
                        if (auto nodeGraphOutput = nodeGraph->getOutput(outputName)) {
                            // We instantiate this node graph in the current node.
                            //
                            auto subNodeIt = subNodes.find(nodeGraph->getName());
                            if (subNodeIt != subNodes.end()) {
                                status = subNodeIt->second->Connect(nodeGraphOutput.get(), this, bindInput.get(), context);
                            } else {
                                try {
                                    auto subNode = std::make_unique<MtlxNodeGraphNode>(std::move(nodeGraph), context);
                                    status = subNode->Connect(nodeGraphOutput.get(), this, bindInput.get(), context);
                                    if (status == RPR_SUCCESS) {
                                        subNodes[valueElement->getName()] = std::move(subNode);
                                    }
                                } catch (MtlxNodeGraphNode::NoOutputsError&) {
                                    context->LogLine("Error: binded nodegraph %s has no outputs\n", nodeGraph->getName().c_str());
                                }
                            }
                        }
                    }
                } else {
                    // A global output is an output that is defined globally in mtlxDocument
                    //
                    if (auto globalOutput = context->mtlxDocument->getOutput(outputName)) {
                        // Such outputs point to a globally instantiated nodes
                        //
                        if (auto mtlxGlobalNode = globalOutput->getConnectedNode()) {
                            if (auto mxtlGlobalNodeDef = mtlxGlobalNode->getNodeDef()) {
                                if (auto mtlxGlobalNodeOutput = GetOutput(mxtlGlobalNodeDef.get(), globalOutput.get(), context)) {
                                    if (auto globalNode = context->GetGlobalNode(mtlxGlobalNode.get())) {
                                        status = globalNode->Connect(mtlxGlobalNodeOutput.get(), this, bindInput.get(), context);
                                    }
                                }
                            }
                        }
                    }
                }

                if (status == RPR_SUCCESS) {
                    continue;
                }
            }
        }

        // If we get here then the input value was not set yet, check `value` attribute.
        //
        auto& valueStr = valueElement->getValueString();
        if (!valueStr.empty()) {
            context->LogLine("%s\n", valueStr.c_str());

            status = node->SetInput(inputElement.get(), valueStr, valueElement->getType(), context);
            if (status == RPR_SUCCESS) {
                continue;
            }
        }

        if (status == RPR_SUCCESS) {
            context->LogLine("Error: invalid %s structure: no value specified - %s\n", valueElement->getCategory().c_str(), valueElement->asString().c_str());
        }
    }

    return node;
}

rpr_status MtlxNodeGraphNode::Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) {
    mx::OutputPtr output;
    if (outputElement) {
        output = mtlxGraph->getOutput(outputElement->getName());
    } else {
        output = GetFirst<mx::Output>(mtlxGraph.get());
    }

    if (output && !output->getNodeName().empty()) {
        auto nodeIt = subNodes.find(output->getNodeName());
        if (nodeIt != subNodes.end()) {
            return nodeIt->second->Connect(nullptr, downstreamNode, inputElement, context);
        }
    }

    return RPR_ERROR_INVALID_PARAMETER;
}

MtlxNodeGraphNode::InterfaceSocketQuery MtlxNodeGraphNode::GetInterfaceSocket(mx::Element* inputElement) {
    auto interfaceSocketIt = _interfaceSockets.find(inputElement->getName());
    if (interfaceSocketIt != _interfaceSockets.end()) {
        auto& interfaceSocket = interfaceSocketIt->second;
        auto subNodeIt = subNodes.find(interfaceSocket.subNode->getName());
        if (subNodeIt != subNodes.end()) {
            InterfaceSocketQuery query;
            query.node = subNodeIt->second.get();
            query.inputElement = interfaceSocket.input.get();
            return query;
        }
    }

    return {};
}

rpr_status MtlxNodeGraphNode::SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) {
    auto socket = GetInterfaceSocket(inputElement);
    if (socket.node && socket.inputElement) {
        return socket.node->SetInput(socket.inputElement, inputNode, context);
    }

    context->LogLine("Error: failed to set %s input for %s: no such interface socket\n", inputElement->getName().c_str(), mtlxGraph->getName().c_str());
    return RPR_ERROR_INVALID_PARAMETER;
}

rpr_status MtlxNodeGraphNode::SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) {
    auto socket = GetInterfaceSocket(inputElement);
    if (socket.node && socket.inputElement) {
        return socket.node->SetInput(socket.inputElement, value, valueType, context);
    }

    context->LogLine("Error: failed to set %s input for %s: no such interface socket\n", inputElement->getName().c_str(), mtlxGraph->getName().c_str());
    return RPR_ERROR_INVALID_PARAMETER;
}

//------------------------------------------------------------------------------
// RprNode implementation
//------------------------------------------------------------------------------

RprNode::RprNode(rpr_material_node node, bool retainNode)
    : isOwningRprNode(retainNode), rprNode(node) {

}

RprNode::~RprNode() {
    if (rprNode && isOwningRprNode) {
        rprObjectDelete(rprNode);
    }
}

rpr_status RprNode::Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) {
    // Ignoring outputElement because rprNode is the only possible output
    //
    return downstreamNode->SetInput(inputElement, rprNode, context);
}

rpr_status RprNode::SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) {
    return RPR_ERROR_UNSUPPORTED;
}

rpr_status RprNode::SetInput(mx::Element* inputElement, std::string const& value, std::string const& valueType, LoaderContext* context) {
    return RPR_ERROR_UNSUPPORTED;
}

rpr_status RprNode::SetInput(rpr_material_node_input inputId, std::string const& valueString, std::string const& valueType, LoaderContext* context) {
    try {
        if (valueType == "float") {
            auto value = mx::fromValueString<float>(valueString);
            return rprMaterialNodeSetInputFByKey(rprNode, inputId, value, value, value, 0.0f);
        } else if (
            valueType == "color2" ||
            valueType == "vector2") {
            auto value = mx::fromValueString<mx::Color2>(valueString);
            return rprMaterialNodeSetInputFByKey(rprNode, inputId, value[0], value[1], 0.0f, 0.0f);
        } else if (
            valueType == "color3" ||
            valueType == "vector3") {
            auto value = mx::fromValueString<mx::Color3>(valueString);
            return rprMaterialNodeSetInputFByKey(rprNode, inputId, value[0], value[1], value[2], 0.0f);
        } else if (
            valueType == "boolean") {
            auto value = static_cast<float>(mx::fromValueString<bool>(valueString));
            return rprMaterialNodeSetInputFByKey(rprNode, inputId, value, value, value, 0.0f);
        } else if (
            valueType == "integer") {
            auto value = static_cast<float>(mx::fromValueString<int>(valueString));
            return rprMaterialNodeSetInputFByKey(rprNode, inputId, value, value, value, 0.0f);
        } else {
            context->LogLine("Error: failed to parse %s value: unsupported type - %s\n", valueString.c_str(), valueType.c_str());
        }
    } catch (mx::ExceptionTypeError& e) {
        context->LogLine("Error: failed to parse %s value: %s\n", valueString.c_str(), e.what());
    }

    return RPR_ERROR_INVALID_PARAMETER;
}

RprMappedNode::RprMappedNode(rpr_material_node node, Mtlx2Rpr::Node const* nodeMapping)
    : RprNode(node, true), rprNodeMapping(nodeMapping) {

}

rpr_status RprMappedNode::SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) {
    auto inputIt = rprNodeMapping->inputs.find(inputElement->getName());
    if (inputIt == rprNodeMapping->inputs.end()) {
        context->LogLine("Error: unknown input: %s\n", inputElement->getName().c_str());
        return RPR_ERROR_INVALID_PARAMETER;
    }

    return rprMaterialNodeSetInputNByKey(rprNode, inputIt->second, inputNode);
}

rpr_status RprMappedNode::SetInput(mx::Element* inputElement, std::string const& valueString, std::string const& valueType, LoaderContext* context) {
    auto inputIt = rprNodeMapping->inputs.find(inputElement->getName());
    if (inputIt == rprNodeMapping->inputs.end()) {
        context->LogLine("Error: unknown input: %s\n", inputElement->getName().c_str());
        return RPR_ERROR_INVALID_PARAMETER;
    }

    return RprNode::SetInput(inputIt->second, valueString, valueType, context);
}

} // namespace anonymous

//------------------------------------------------------------------------------
// Loader
//------------------------------------------------------------------------------

HsMtlxLoader::Result HsMtlxLoader::Load(mx::Document* mtlxDocument, rpr_material_system rprMatSys) {
    // Import of stdlib should be postponed as further as possible
    // because of how we look for target nodegraph.
    // At first we look up for the very first material, then if not available,
    // for the very first node def and then same for nodegraph.
    // If we will import stdlib asap we will query nodegraphs and nodedefs from it as well
    // (not just from the original mtlxDocument) which is undesirable.
    // Such logic is unneeded if we know what exactly we want to use as root material graph.
    bool stdlibIsImported = false;
    auto importStdlib = [&stdlibIsImported, &mtlxDocument, this]() {
        if (!stdlibIsImported) {
            stdlibIsImported = true;
            mtlxDocument->importLibrary(_stdlib);
        }
    };

    mx::ShaderRefPtr shaderRef;
    mx::GraphElementPtr nodeGraph;

    {
        mx::NodeDefPtr nodeDef;

        // TODO: expose the way to control which material we want to use from .mtlx
        if (auto material = GetFirst<mx::Material>(mtlxDocument)) {
            shaderRef = GetFirst<mx::ShaderRef>(material.get());
            if (shaderRef) {
                importStdlib();
                nodeDef = shaderRef->getNodeDef();
            }
        } else {
            // If there is no material in the file, use one of the present node definitions
            //
            nodeDef = GetFirst<mx::NodeDef>(mtlxDocument);
        }

        if (nodeDef) {
            if (auto impl = nodeDef->getImplementation()) {
                nodeGraph = impl->asA<mx::NodeGraph>();
            }
        } else {
            nodeGraph = GetFirst<mx::NodeGraph>(mtlxDocument);
        }

        // If there are no materials, nodeDefs or nodeGraphs in the file,
        // use the current document as nodeGraph
        //
        if (!nodeGraph) {
            nodeGraph = mtlxDocument->shared_from_this()->asA<mx::GraphElement>();
        }
    }

    bool hasAnyOutput = false;
    mx::OutputPtr outputs[kMaxNumOutputs];
    for (auto& child : nodeGraph->getChildren()) {
        if (auto output = child->asA<mx::Output>()) {
            if (output->getType() == "surfaceshader") {
                outputs[Surface] = std::move(output);
                hasAnyOutput = true;
            } else if (output->getType() == "displacementshader") {
                outputs[Displacement] = std::move(output);
                hasAnyOutput = true;
            }
        }
    }
    if (!hasAnyOutput) {
        if (_loggingEnabled) {
            printf("No renderable elements in %s\n", mtlxDocument->getSourceUri().c_str());
        }
        return {};
    }

    LoaderContext ctx = {};
    ctx.logEnabled = _loggingEnabled;
    ctx.mtlxDocument = mtlxDocument;
    ctx.rprMatSys = rprMatSys;
    ctx.shaderRef = shaderRef.get();

    importStdlib();
    MtlxNodeGraphNode graphNode(nodeGraph, &ctx);

    hasAnyOutput = false;
    RprNode* rprOutputs[kMaxNumOutputs] = {};
    for (int i = 0; i < kMaxNumOutputs; ++i) {
        if (!outputs[i]) {
            continue;
        }

        auto it = graphNode.subNodes.find(outputs[i]->getNodeName());
        if (it != graphNode.subNodes.end()) {
            if (auto rprNode = it->second->AsA<RprNode>()) {
                hasAnyOutput = true;
                rprOutputs[i] = rprNode;
            }
        }
    }
    if (!hasAnyOutput) {
        return {};
    }

    // Convert graphNode to Result

    Result ret = {};

    // Calculate the total number of rpr nodes
    //
    ctx.TraverseNodes(&graphNode, [&ret](Node* node) {
        if (auto rprNode = node->AsA<RprNode>()) {
            if (rprNode->isOwningRprNode) {
                ret.numNodes++;
            }
        }
    });

    ret.nodes = new rpr_material_node[ret.numNodes];

    // Map rpr_material_node handles to output type. We use it to fill rootNodeIndices
    //
    std::unordered_map<rpr_material_node, OutputType> targetOutputs;
    for (int i = 0; i < kMaxNumOutputs; ++i) {
        if (rprOutputs[i] && rprOutputs[i]->rprNode) {
            targetOutputs.emplace(rprOutputs[i]->rprNode, static_cast<OutputType>(i));
        }
    }

    // Move all rpr nodes from subnodes into the return array
    //
    size_t nodeIdx = 0;
    ctx.TraverseNodes(&graphNode, [&ret, &nodeIdx](Node* node) {
        if (auto rprNode = node->AsA<RprNode>()) {
            // Move only unique nodes
            //
            if (rprNode->isOwningRprNode) {
                ret.nodes[nodeIdx++] = rprNode->rprNode;
                rprNode->rprNode = nullptr;
            }
        }
    });

    // Fill root node indices
    //
    for (int i = 0; i < kMaxNumOutputs; ++i) {
        ret.rootNodeIndices[i] = Result::kInvalidRootNodeIndex;
    }
    for (size_t i = 0; i < ret.numNodes; ++i) {
        auto it = targetOutputs.find(ret.nodes[i]);
        if (it != targetOutputs.end()) {
            ret.rootNodeIndices[it->second] = i;
            targetOutputs.erase(it);
        }
    }

    if (!ctx.imageNodes.empty()) {
        ret.numImageNodes = ctx.imageNodes.size();
        ret.imageNodes = new Result::ImageNode[ret.numImageNodes];
        for (size_t i = 0; i < ret.numImageNodes; ++i) {
            ret.imageNodes[i] = ctx.imageNodes[i];
        }
    }

    return ret;
}

void HsMtlxLoader::SetupStdlib(MaterialX::FilePathVec const& libraryNames, MaterialX::FileSearchPath const& searchPath) {
    _stdlib = mx::createDocument();
    mx::loadLibraries(libraryNames, searchPath, _stdlib);
}
