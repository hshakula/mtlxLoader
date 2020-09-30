#include "hsMtlxLoader.h"

#include <MaterialXFormat/Util.h> // mx::loadLibraries

#include <map>
#include <cstdarg>

namespace mx = MaterialX;

namespace {

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

        // Why?
        arithmeticOps["surface"] = RPR_MATERIAL_NODE_OP_ADD;
        nodes["surface"] = {
            RPR_MATERIAL_NODE_ARITHMETIC, {
                {"bsdf", RPR_MATERIAL_INPUT_COLOR0},
                {"edf", RPR_MATERIAL_INPUT_COLOR1},
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

struct LoaderContext {
    mx::Node* nodeRef;
    mx::ShaderRef* shaderRef;
    mx::Document* mtlxDocument;
    rpr_material_system rprMatSys;

    std::vector<HsMtlxLoader::Result::ImageNode> imageNodes;

    std::map<std::string, std::unique_ptr<Node>> globalNodes;
    Node* GetGlobalNode(mx::Element* element);

    int logDepth = 0;
    bool logEnabled = true;

    void LogLine(const char* fmt, ...) {
        if (logEnabled) {
            if (logDepth > 0) {
                printf("%*s", logDepth * 3, "");
            }

            va_list ap;
            va_start(ap, fmt);
            vprintf(fmt, ap);
            va_end(ap);
        }
    }

    struct NestedLogScope {
        LoaderContext* ctx;

        NestedLogScope(LoaderContext* ctx) : ctx(ctx) { ctx->logDepth++; }
        ~NestedLogScope() { ctx->logDepth--; }
    };
    NestedLogScope EnterNestedLogScope() { return NestedLogScope(this); };

    struct GlobalLogScope {
        LoaderContext* ctx;
        int prevDepth;

        GlobalLogScope(LoaderContext* ctx) : ctx(ctx), prevDepth(ctx->logDepth) { ctx->logDepth = 0; }
        ~GlobalLogScope() { ctx->logDepth = prevDepth; }
    };
    GlobalLogScope EnterGlobalLogScope() { return GlobalLogScope(this); };

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
        for (auto& entry : node->subNodes) {
            TraverseSubNodes(entry.second.get(), cb);
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
    std::map<std::string, Node::Ptr> subNodes;

    virtual ~Node() = default;

    /// Connect the current (upstream) node to the \p downstreamNode
    virtual rpr_status Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) = 0;

    virtual rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) = 0;
    virtual rpr_status SetInput(mx::Element* inputElement, mx::ValueElement* value, LoaderContext* context) = 0;

    template <typename T>
    T* AsA() {
        return dynamic_cast<T*>(this);
    }

    static Node::Ptr Create(mx::Element* element, LoaderContext* context);
};

/// The node with rpr_material_node underlying type
///
struct RprNode : public Node {
    rpr_material_node rprNode;
    Mtlx2Rpr::Node const* rprNodeMapping;

    /// RprNode takes ownership over \p node
    RprNode(rpr_material_node node, Mtlx2Rpr::Node const* nodeMapping);
    ~RprNode() override;

    rpr_status Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, mx::ValueElement* value, LoaderContext* context) override;
};

/// The node with mx::NodeGraph underlying type
///
struct MtlxNodeGraphNode : public Node {
    mx::NodeGraphPtr mtlxGraph;

    MtlxNodeGraphNode(mx::NodeGraphPtr mtlxGraph, LoaderContext* context);
    ~MtlxNodeGraphNode() override = default;

    rpr_status Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) override;
    rpr_status SetInput(mx::Element* inputElement, mx::ValueElement* value, LoaderContext* context) override;

    Node* GetNode(mx::Element* element, LoaderContext* context);
};

//------------------------------------------------------------------------------
// Loader context implementation
//------------------------------------------------------------------------------

Node* LoaderContext::GetGlobalNode(mx::Element* element) {
    auto it = globalNodes.find(element->getName());
    if (it == globalNodes.end()) {
        auto logScope = EnterGlobalLogScope();
        if (auto newNode = Node::Create(element, this)) {
            it = globalNodes.emplace(element->getName(), std::move(newNode)).first;
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
Node::Ptr Node::Create(mx::Element* element, LoaderContext* context) {
    // TODO: Is it possible to have here something except node/output?
    auto mtlxNode = element->asA<mx::Node>();
    if (!mtlxNode) {
        if (element->getCategory() != "output") {
            context->LogLine("Unexpected element type: %s (%s)\n", element->getName().c_str(), element->getCategory().c_str());
        }
        return nullptr;
    }

    context->LogLine(" - Node: %s (%s)\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str());

    Node::Ptr newNode;
    rpr_material_node rprNode = nullptr;
    Mtlx2Rpr::Node const* rprNodeMapping = nullptr;

    // Check for nodes with special handling first
    //
    if (mtlxNode->getCategory() == "texcoord") {
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
            // Some of the standard nodes can be mapped to RPR nodes directly
            //
            auto rprNodeMappingIt = GetMtlx2Rpr().nodes.find(mtlxNode->getCategory());
            if (rprNodeMappingIt != GetMtlx2Rpr().nodes.end()) {
                rprNodeMapping = &rprNodeMappingIt->second;

            // But direct mapping might not have been implemented yet or
            // this node might be of a custom definition
            //
            } else {
                // Check if this node has a custom definition
                //
                for (auto& nodeDef : context->mtlxDocument->getMatchingNodeDefs(mtlxNode->getCategory())) {
                    if (auto implementation = nodeDef->getImplementation()) {
                        if (auto nodeGraph = implementation->asA<mx::NodeGraph>()) {
                            auto nodeRefOver = context->OverrideNodeRef(mtlxNode.get());
                            newNode = std::make_unique<MtlxNodeGraphNode>(std::move(nodeGraph), context);
                            break;
                        } else {
                            // TODO: code generation required
                        }
                    }
                }

                if (!newNode) {
                    context->LogLine("Unsupported node: %s (%s)\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str());
                }
            }

        }
    }

    if (!rprNode && rprNodeMapping) {
        auto status = rprMaterialSystemCreateNode(context->rprMatSys, rprNodeMapping->id, &rprNode);
        if (status != RPR_SUCCESS) {
            context->LogLine("Failed to create %s (%s) node: %d\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str(), status);
            return nullptr;
        }

        // For arithmetic nodes, we also must not forget to set operation
        //
        if (rprNodeMapping->id == RPR_MATERIAL_NODE_ARITHMETIC) {
            auto it = GetMtlx2Rpr().arithmeticOps.find(mtlxNode->getCategory());
            if (it != GetMtlx2Rpr().arithmeticOps.end()) {
                rprMaterialNodeSetInputUByKey(rprNode, RPR_MATERIAL_INPUT_OP, it->second);
            } else {
                context->LogLine("Unknown arithmetic node: %s (%s)", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str());
            }
        }
    }

    // Set rpr node inputs, nodes of other types set their inputs on creation
    //
    if (rprNode) {
        newNode = std::make_unique<RprNode>(rprNode, rprNodeMapping);
        rprObjectSetName(rprNode, mtlxNode->getName().c_str());

        if (rprNodeMapping) {
            for (auto& child : mtlxNode->getChildren()) {
                auto inputElement = child->asA<mx::ValueElement>();
                if (!inputElement) {
                    continue;
                }

                context->LogLine("  - %s: %s\n", inputElement->getCategory().c_str(), inputElement->getName().c_str());

                auto inputIt = rprNodeMapping->inputs.find(inputElement->getName());
                if (inputIt == rprNodeMapping->inputs.end()) {
                    context->LogLine("    Unknown input for %s (%s) node: %s\n", mtlxNode->getName().c_str(), mtlxNode->getCategory().c_str(), inputElement->getName().c_str());
                    continue;
                }

                mx::ValueElementPtr valueElement;

                // If this input has an interface name then shaderRef or nodeRef might have overridden the default value
                //
                auto& interfaceName = inputElement->getInterfaceName();
                if (!interfaceName.empty()) {
                    mx::ValueElementPtr interfaceValueElement;

                    if (context->nodeRef) {
                        interfaceValueElement = context->nodeRef->getInput(interfaceName);
                    }

                    if (!interfaceValueElement && context->shaderRef) {
                        interfaceValueElement = context->shaderRef->getBindInput(interfaceName);
                    }

                    if (!interfaceValueElement) {
                        // Otherwise, get default value from node definition
                        //
                        if (auto nodeGraph = mtlxNode->getAncestorOfType<mx::NodeGraph>()) {
                            if (auto nodeDef = nodeGraph->getNodeDef()) {
                                interfaceValueElement = nodeDef->getValueElement(interfaceName);
                            }
                        }
                    }

                    if (interfaceValueElement) {
                        valueElement = interfaceValueElement;
                    }
                }

                if (!valueElement && !inputElement->getValueString().empty()) {
                    valueElement = inputElement;
                }

                if (valueElement) {
                    newNode->SetInput(inputElement.get(), valueElement.get(), context);
                } else {
                    context->LogLine("    No value\n");
                }
            }
        }
    }

    return newNode;
}

//------------------------------------------------------------------------------
// MtlxNodeGraphNode implementation
//------------------------------------------------------------------------------

MtlxNodeGraphNode::MtlxNodeGraphNode(mx::NodeGraphPtr graph, LoaderContext* context)
    : mtlxGraph(std::move(graph)) {

    context->LogLine("- NodeGraph: %s\n", mtlxGraph->getName().c_str());
    auto NestedLogScope = context->EnterNestedLogScope();

    std::set<mx::Edge> processedEdges;
    for (auto& output : mtlxGraph->getOutputs()) {
        for (auto& edge : output->traverseGraph()) {
            if (processedEdges.count(edge)) {
                continue;
            }
            processedEdges.insert(edge);

            auto upstreamElem = edge.getUpstreamElement();
            auto downstreamElem = edge.getDownstreamElement();
            if (!upstreamElem || !downstreamElem) continue;

            auto upstreamNode = GetNode(upstreamElem.get(), context);
            if (!upstreamNode) {
                context->LogLine("Failed to process %s edge: upstream node does not exist\n", to_string(edge).c_str());
                continue;
            }

            auto connectingElem = edge.getConnectingElement();
            if (!connectingElem) {
                continue;
            }

            auto downstreamNode = GetNode(downstreamElem.get(), context);
            if (!downstreamNode) {
                context->LogLine("Failed to process %s edge: downstream node does not exist\n", to_string(edge).c_str());
                continue;
            }

            auto status = upstreamNode->Connect(nullptr, downstreamNode, connectingElem.get(), context);
            if (status == RPR_SUCCESS) {
                context->LogLine(" - Connection: %s\n", to_string(edge).c_str());

            // TODO: should we invalidate node here?
            } else if (status == RPR_ERROR_INVALID_PARAMETER) {
                context->LogLine(" - Failed to connect %s edge: downstream node does not have such input\n", to_string(edge).c_str());
            } else {
                context->LogLine(" - Failed to connect %s edge: rpr error - %d\n", to_string(edge).c_str(), status);
            }
        }
    }
}

Node* MtlxNodeGraphNode::GetNode(mx::Element* element, LoaderContext* context) {
    // Return cached node if such present
    //
    auto it = subNodes.find(element->getName());
    if (it != subNodes.end()) {
        return it->second.get();
    }

    auto newNode = Node::Create(element, context);

    auto retPtr = newNode.get();
    if (newNode) {
        subNodes[element->getName()] = std::move(newNode);
    }
    return retPtr;
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

rpr_status MtlxNodeGraphNode::SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) {
    // TODO: Is it possible?
    context->LogLine(" - Failed to set input for %s: unimplemented\n", mtlxGraph->getName().c_str());
    return RPR_ERROR_UNIMPLEMENTED;
}

rpr_status MtlxNodeGraphNode::SetInput(mx::Element* inputElement, mx::ValueElement* value, LoaderContext* context) {
    // TODO: Is it possible?
    context->LogLine(" - Failed to set %s input for %s: unimplemented\n", value->asString().c_str(), mtlxGraph->getName().c_str());
    return RPR_ERROR_UNIMPLEMENTED;
}

//------------------------------------------------------------------------------
// RprNode implementation
//------------------------------------------------------------------------------

RprNode::RprNode(rpr_material_node node, Mtlx2Rpr::Node const* nodeMapping)
    : rprNode(node), rprNodeMapping(nodeMapping) {

}

RprNode::~RprNode() {
    if (rprNode) {
        rprObjectDelete(rprNode);
    }
}

rpr_status RprNode::Connect(mx::Element* outputElement, Node* downstreamNode, mx::Element* inputElement, LoaderContext* context) {
    // Ignoring outputElement because rprNode is the only possible output
    //
    return downstreamNode->SetInput(inputElement, rprNode, context);
}

rpr_status RprNode::SetInput(mx::Element* inputElement, rpr_material_node inputNode, LoaderContext* context) {
    // RprNode without mapping does not expect any inputs to be set
    if (!rprNodeMapping) {
        return RPR_ERROR_UNSUPPORTED;
    }

    auto inputIt = rprNodeMapping->inputs.find(inputElement->getName());
    if (inputIt == rprNodeMapping->inputs.end()) {
        return RPR_ERROR_INVALID_PARAMETER;
    }

    return rprMaterialNodeSetInputNByKey(rprNode, inputIt->second, inputNode);
}

rpr_status RprNode::SetInput(mx::Element* inputElement, mx::ValueElement* value, LoaderContext* context) {
    // RprNode without mapping does not expect any inputs to be set
    if (!rprNodeMapping) {
        return RPR_ERROR_UNSUPPORTED;
    }

    auto inputIt = rprNodeMapping->inputs.find(inputElement->getName());
    if (inputIt == rprNodeMapping->inputs.end()) {
        return RPR_ERROR_INVALID_PARAMETER;
    }

    // The value element may specify either value or output but not both at once
    //
    if (!value->getValueString().empty()) {
        context->LogLine("    %s\n", value->getValueString().c_str());

        auto& valueStr = value->getValueString();
        try {
            if (value->getType() == "float") {
                auto value = mx::fromValueString<float>(valueStr);
                return rprMaterialNodeSetInputFByKey(rprNode, inputIt->second, value, value, value, 0.0f);
            } else if (
                value->getType() == "color2" ||
                value->getType() == "vector2") {
                auto value = mx::fromValueString<mx::Color2>(valueStr);
                return rprMaterialNodeSetInputFByKey(rprNode, inputIt->second, value[0], value[1], 0.0f, 0.0f);
            } else if (
                value->getType() == "color3" ||
                value->getType() == "vector3") {
                auto value = mx::fromValueString<mx::Color3>(valueStr);
                return rprMaterialNodeSetInputFByKey(rprNode, inputIt->second, value[0], value[1], value[2], 0.0f);
            } else if (
                value->getType() == "boolean") {
                auto value = static_cast<float>(mx::fromValueString<bool>(valueStr));
                return rprMaterialNodeSetInputFByKey(rprNode, inputIt->second, value, value, value, 0.0f);
            } else if (
                value->getType() == "integer") {
                auto value = static_cast<float>(mx::fromValueString<int>(valueStr));
                return rprMaterialNodeSetInputFByKey(rprNode, inputIt->second, value, value, value, 0.0f);
            } else {
                context->LogLine("    Failed to parse %s value: unsupported type - %s\n", value->getName().c_str(), value->getType().c_str());
            }
        } catch (mx::ExceptionTypeError& e) {
            context->LogLine("    Failed to parse %s value: %s\n", value->getName().c_str(), e.what());
        }
    } else {
        auto& outputName = value->getAttribute(mx::PortElement::OUTPUT_ATTRIBUTE);
        if (!outputName.empty()) {

            // The output string may point to the output of some node graph or free-standing (global) output
            //
            auto& nodeGraphName = value->getAttribute(mx::PortElement::NODE_GRAPH_ATTRIBUTE);
            if (!nodeGraphName.empty()) {
                if (auto nodeGraph = context->mtlxDocument->getNodeGraph(nodeGraphName)) {
                    if (auto nodeGraphOutput = nodeGraph->getOutput(outputName)) {
                        auto subNode = std::make_unique<MtlxNodeGraphNode>(std::move(nodeGraph), context);
                        auto status = subNode->Connect(nodeGraphOutput.get(), this, inputElement, context);
                        if (status == RPR_SUCCESS) {
                            subNodes[value->getName()] = std::move(subNode);
                            return RPR_SUCCESS;
                        }
                    }
                }

            // A global output is an output that is defined globally in mtlxDocument
            //
            } else {
                if (auto output = context->mtlxDocument->getOutput(outputName)) {
                    // Such outputs point to a globally instantiated nodes
                    //
                    if (auto connectedNode = output->getConnectedNode()) {
                        auto connectedNodeOutputName = output->getOutputString();
                        if (auto connectedNodeDef = connectedNode->getNodeDef()) {
                            if (auto connectedNodeOutput = connectedNodeDef->getOutput(connectedNodeOutputName)) {
                                if (auto globalNode = context->GetGlobalNode(connectedNode.get())) {
                                    return globalNode->Connect(connectedNodeOutput.get(), this, inputElement, context);
                                }
                            }
                        }
                    }
                }
            }
        } else {
            context->LogLine("    Failed to set input: invalid value element structure - %s\n", value->asString().c_str());
        }
    }

    return RPR_ERROR_INVALID_PARAMETER;
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
    mx::NodeGraphPtr nodeGraph;

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
    }

    importStdlib();

    if (nodeGraph) {
        // TODO: add other shader types
        mx::OutputPtr surfaceOutput;
        for (auto& child : nodeGraph->getChildren()) {
            if (auto output = child->asA<mx::Output>()) {
                if (output->getType() == "surfaceshader") {
                    surfaceOutput = std::move(output);
                    break;
                }
            }
        }
        if (!surfaceOutput) {
            return {};
        }

        LoaderContext ctx = {};
        ctx.logEnabled = _loggingEnabled;
        ctx.mtlxDocument = mtlxDocument;
        ctx.rprMatSys = rprMatSys;
        ctx.shaderRef = shaderRef.get();
        MtlxNodeGraphNode graphNode(nodeGraph, &ctx);

        auto surfaceNodeIt = graphNode.subNodes.find(surfaceOutput->getNodeName());
        if (surfaceNodeIt != graphNode.subNodes.end()) {
            RprNode* rootNode = nullptr;

            // XXX (Norhtstar): `surface` node can not be used as root node, it may lead to corrupted renders
            auto rootMtlxNode = nodeGraph->getNode(surfaceOutput->getNodeName());
            if (rootMtlxNode->getCategory() == "surface") {
                if (auto bsdfInput = rootMtlxNode->getInput("bsdf")) {
                    auto actualRootNodeIt = graphNode.subNodes.find(bsdfInput->getNodeName());
                    if (actualRootNodeIt != graphNode.subNodes.end()) {
                        rootNode = actualRootNodeIt->second->AsA<RprNode>();
                    }
                }
            } else {
                rootNode = surfaceNodeIt->second->AsA<RprNode>();
            }

            if (rootNode) {

                Result ret = {};

                // Calculate the total number of rpr nodes
                //
                ctx.TraverseNodes(&graphNode, [&ret](Node* node) {
                    if (auto rprNode = node->AsA<RprNode>()) {
                        ret.numNodes++;
                    }
                });

                ret.nodes = new rpr_material_node[ret.numNodes];

                // Move all rpr nodes from subnodes into the return array
                //
                size_t nodeIdx = 0;
                ctx.TraverseNodes(&graphNode, [&ret, &nodeIdx, &rootNode](Node* node) {
                    if (auto rprNode = node->AsA<RprNode>()) {
                        if (rprNode->rprNode == rootNode->rprNode) {
                            ret.surfaceRootNodeIdx = nodeIdx;
                        }
                        ret.nodes[nodeIdx++] = rprNode->rprNode;
                        rprNode->rprNode = nullptr;
                    }
                });

                if (!ctx.imageNodes.empty()) {
                    ret.numImageNodes = ctx.imageNodes.size();
                    ret.imageNodes = new Result::ImageNode[ret.numImageNodes];
                    for (size_t i = 0; i < ret.numImageNodes; ++i) {
                        ret.imageNodes[i] = ctx.imageNodes[i];
                    }
                }

                return ret;
            }
        }
    }

    return {};
}

void HsMtlxLoader::SetupStdlib(MaterialX::FilePathVec const& libraryNames, MaterialX::FileSearchPath const& searchPath) {
    _stdlib = mx::createDocument();
    mx::loadLibraries(libraryNames, searchPath, _stdlib);
}
