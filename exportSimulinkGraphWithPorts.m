function graph = exportSimulinkGraphWithPorts(modelName)
    loadModelIfNeeded(modelName);
    blocks = getFilteredBlocks(modelName);
    [~, nodeList, edgeList] = buildGraphFromBlocks(blocks);
    graph = finalizeGraph(nodeList, edgeList);
    writeGraphToJSON(graph, modelName);
end


function loadModelIfNeeded(modelName)
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
end

function blocks = getFilteredBlocks(modelName)
    blocks = find_system(modelName, ...
        'FollowLinks', 'on', ...
        'LookUnderMasks', 'all', ...
        'SearchDepth', Inf, ...
        'Type', 'Block');
end

function [nodeMap, nodeList, edgeList] = buildGraphFromBlocks(blocks)
    edgeList = [];
    nodeMap = containers.Map();
    nodeList = {};
    nextId = 1;

    for i = 1:length(blocks)
        block = blocks{i};
        blockName = getfullname(block);
        blockType = get_param(block, 'BlockType');

        % Assign ID to block node
        if ~isKey(nodeMap, blockName)
            nodeMap(blockName) = nextId;
            nodeList{nextId} = struct('id', nextId, 'name', blockName, 'type', blockType, 'nodeType', 'block');
            nextId = nextId + 1;
        end

        ports = get_param(block, 'PortHandles');
        [nodeMap, nodeList, edgeList, nextId] = addInputEdges(blockName, ports, nodeMap, nodeList, edgeList, nextId);
        [nodeMap, nodeList, edgeList, nextId] = addOutputEdges(blockName, ports, nodeMap, nodeList, edgeList, nextId);
    end
end

function [nodeMap, nodeList, edgeList, nextId] = addInputEdges(blockName, ports, nodeMap, nodeList, edgeList, nextId)
    blockType = get_param(blockName, 'BlockType');
    isSubsystem = strcmp(blockType, 'SubSystem');
    if isSubsystem
        return;
    end
    if isfield(ports, 'Inport')
        for p = 1:length(ports.Inport)
            inNodeName = sprintf('%s_in%d', blockName, p);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, inNodeName, blockName, p, 'input', nextId);

            % Skip edge to subsystem block
            if ~isSubsystem
                edgeList = addEdge(edgeList, nodeMap(inNodeName), nodeMap(blockName));
            end

            line = get_param(ports.Inport(p), 'Line');
            [nodeMap, nodeList, edgeList, nextId] = handleInputLineSource( ...
                inNodeName, line, nodeMap, nodeList, edgeList, nextId);
        end
    end

    % Handle control ports (Enable, Trigger, Action) if needed
end

function [nodeMap, nodeList, edgeList, nextId] = handleInputLineSource(inNodeName, line, nodeMap, nodeList, edgeList, nextId)
    if line ~= -1 && line ~= 0
        srcPort = get_param(line, 'SrcPortHandle');
        srcBlock = get_param(srcPort, 'Parent');
        srcBlockType = get_param(srcBlock, 'BlockType');

        portNum = get_param(srcPort, 'PortNumber');

        if strcmp(srcBlockType, 'SubSystem') || strcmp(srcBlockType, 'Outport')
            realSrcPorts = traceThroughOutport(srcBlock, portNum);
            for k = 1:length(realSrcPorts)
                realSrcBlock = get_param(realSrcPorts{k}, 'Parent');
                realSrcBlockName = getfullname(realSrcBlock);
                realSrcPortNum = get_param(realSrcPorts{k}, 'PortNumber');
                outNodeName = sprintf('%s_out%d', realSrcBlockName, realSrcPortNum);

                [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                    nodeMap, nodeList, outNodeName, realSrcBlockName, realSrcPortNum, 'output', nextId);
                edgeList = addEdge(edgeList, nodeMap(outNodeName), nodeMap(inNodeName));
            end
        else
            srcBlockName = getfullname(srcBlock);
            outNodeName = sprintf('%s_out%d', srcBlockName, portNum);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, outNodeName, srcBlockName, portNum, 'output', nextId);
            edgeList = addEdge(edgeList, nodeMap(outNodeName), nodeMap(inNodeName));
        end
    end
end

function edgeList = addEdge(edgeList, sourceId, targetId)
    edgeList(end+1, :) = [sourceId, targetId];
end

function [nodeMap, nodeList, nextId] = createPortNodeIfNeeded(nodeMap, nodeList, nodeName, blockName, portNum, portType, nextId)
    if ~isKey(nodeMap, nodeName)
        nodeMap(nodeName) = nextId;
        nodeList{nextId} = struct( ...
            'id', nextId, ...
            'name', blockName, ...
            'portNumber', portNum, ...
            'type', portType, ...
            'nodeType', 'port');
        nextId = nextId + 1;
    end
end

function [nodeMap, nodeList, edgeList, nextId] = addOutputEdges(blockName, ports, nodeMap, nodeList, edgeList, nextId)
    blockType = get_param(blockName, 'BlockType');
    isSubsystem = strcmp(blockType, 'SubSystem');
    if isSubsystem 
        return;
    end
    if isfield(ports, 'Outport')
        for p = 1:length(ports.Outport)
            outNodeName = sprintf('%s_out%d', blockName, p);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, outNodeName, blockName, p, 'output', nextId);

            % Skip edge from subsystem block
            if ~isSubsystem
                edgeList = addEdge(edgeList, nodeMap(blockName), nodeMap(outNodeName));
            end

            line = get_param(ports.Outport(p), 'Line');
            if line ~= -1 && line ~= 0
                [nodeMap, nodeList, edgeList, nextId] = handleOutputLineDestinations( ...
                    outNodeName, line, nodeMap, nodeList, edgeList, nextId);
            end
        end
    end
end

function [nodeMap, nodeList, edgeList, nextId] = handleOutputLineDestinations(outNodeName, line, nodeMap, nodeList, edgeList, nextId)
    dstPorts = get_param(line, 'DstPortHandle');
    if ~iscell(dstPorts)
        dstPorts = {dstPorts};
    end

    for j = 1:length(dstPorts)
        dstBlock = get_param(dstPorts{j}, 'Parent');
        dstBlockType = get_param(dstBlock, 'BlockType');
        portNum = get_param(dstPorts{j}, 'PortNumber');

        if strcmp(dstBlockType, 'SubSystem') || strcmp(dstBlockType, 'Inport')
            realDstPorts = traceThroughInport(dstBlock, portNum);
            for k = 1:length(realDstPorts)
                realDstBlock = get_param(realDstPorts{k}, 'Parent');
                realDstBlockName = getfullname(realDstBlock);
                realDstPortNum = get_param(realDstPorts{k}, 'PortNumber');
                dstNodeName = sprintf('%s_in%d', realDstBlockName, realDstPortNum);

                [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                    nodeMap, nodeList, dstNodeName, realDstBlockName, realDstPortNum, 'input', nextId);
                edgeList = addEdge(edgeList, nodeMap(outNodeName), nodeMap(dstNodeName));
            end
        else
            dstBlockName = getfullname(dstBlock);
            dstNodeName = sprintf('%s_in%d', dstBlockName, portNum);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, dstNodeName, dstBlockName, portNum, 'input', nextId);
            edgeList = addEdge(edgeList, nodeMap(outNodeName), nodeMap(dstNodeName));
        end
    end
end

function [shouldSkip, traceThrough] = shouldSkipOrTraceBlock(blockType)
    traceThroughTypes = {'Inport', 'Enable', 'Trigger', 'Action'};
    skipTypes = {'Outport'};

    if ismember(blockType, traceThroughTypes)
        shouldSkip = false;
        traceThrough = true;
    elseif ismember(blockType, skipTypes)
        shouldSkip = true;
        traceThrough = false;
    else
        shouldSkip = false;
        traceThrough = false;
    end
end


function graph = finalizeGraph(nodeList, edgeList)
    graph = struct();
    graph.Nodes = nodeList;
    graph.Edges = edgeList;
end

function writeGraphToJSON(graph, modelName)
    % Write standard graph with numeric edges
    jsonStr = jsonencode(graph);
    jsonStr = prettyPrintJSON(jsonStr);
    fileName = [modelName '_graph.json'];
    writeToFile(fileName, jsonStr);

    % Create verbose graph with named edges
    verboseGraph = graph;
    verboseGraph.Edges = createVerboseEdges(graph);
    verboseStr = jsonencode(verboseGraph);
    verboseStr = prettyPrintJSON(verboseStr);
    verboseFileName = [modelName '_graph_verbose.json'];
    writeToFile(verboseFileName, verboseStr);
end

function writeToFile(fileName, content)
    fid = fopen(fileName, 'w');
    if fid == -1
        error('Cannot create JSON file: %s', fileName);
    end
    fprintf(fid, '%s', content);
    fclose(fid);
end

function verboseEdges = createVerboseEdges(graph)
    % Build ID to formatted name map
    idToName = containers.Map('KeyType', 'double', 'ValueType', 'char');
    for i = 1:length(graph.Nodes)
        node = graph.Nodes{i};
        if strcmp(node.nodeType, 'port')
            formattedName = sprintf('%s:%s:%d', node.name, node.type, node.portNumber);
        else
            formattedName = node.name;
        end
        idToName(node.id) = formattedName;
    end

    % Replace numeric IDs with formatted names and include IDs
    verboseEdges = cell(size(graph.Edges, 1), 1);
    for i = 1:size(graph.Edges, 1)
        srcId = graph.Edges(i, 1);
        dstId = graph.Edges(i, 2);
        verboseEdges{i} = struct( ...
            'source', struct('id', srcId, 'name', idToName(srcId)), ...
            'target', struct('id', dstId, 'name', idToName(dstId)) ...
        );
    end
end

function prettyStr = prettyPrintJSON(jsonStr)
    indent = '    ';
    level = 0;
    prettyStr = '';
    inString = false;

    for i = 1:length(jsonStr)
        ch = jsonStr(i);
        switch ch
            case '"'
                prettyStr(end+1) = ch;
                if i == 1 || jsonStr(i-1) ~= '\'
                    inString = ~inString;
                end
            case '{'
                prettyStr(end+1) = ch;
                if ~inString
                    level = level + 1;
                    prettyStr = [prettyStr newline repmat(indent, 1, level)];
                end
            case '}'
                if ~inString
                    level = level - 1;
                    prettyStr = [prettyStr newline repmat(indent, 1, level) ch];
                else
                    prettyStr(end+1) = ch;
                end
            case '['
                prettyStr(end+1) = ch;
                if ~inString
                    level = level + 1;
                    prettyStr = [prettyStr newline repmat(indent, 1, level)];
                end
            case ']'
                if ~inString
                    level = level - 1;
                    prettyStr = [prettyStr newline repmat(indent, 1, level) ch];
                else
                    prettyStr(end+1) = ch;
                end
            case ','
                prettyStr(end+1) = ch;
                if ~inString
                    prettyStr = [prettyStr newline repmat(indent, 1, level)];
                end
            case ':'
                if ~inString
                    prettyStr = [prettyStr ': '];
                else
                    prettyStr(end+1) = ch;
                end
            otherwise
                prettyStr(end+1) = ch;
        end
    end
end



function dstPorts = traceThroughInport(subsystem, portNum)
    dstPorts = {};
    inports = find_system(subsystem, 'SearchDepth', 1, 'BlockType', 'Inport');

    for i = 1:length(inports)
        if str2double(get_param(inports{i}, 'Port')) == portNum
            portHandles = get_param(inports{i}, 'PortHandles');
            outLine = get_param(portHandles.Outport, 'Line');
            if outLine ~= -1 && outLine ~= 0
                nextDstPorts = get_param(outLine, 'DstPortHandle');
                if ~iscell(nextDstPorts)
                    nextDstPorts = {nextDstPorts};
                end
                for j = 1:length(nextDstPorts)
                    dstBlock = get_param(nextDstPorts{j}, 'Parent');
                    dstBlockType = get_param(dstBlock, 'BlockType');
                    if strcmp(dstBlockType, 'SubSystem')
                        nestedPortNum = get_param(nextDstPorts{j}, 'PortNumber');
                        nestedDstPorts = traceThroughInport(dstBlock, nestedPortNum);
                        dstPorts = [dstPorts nestedDstPorts];
                    else
                        dstPorts{end+1} = nextDstPorts{j};
                    end
                end
            end
            break;
        end
    end
end

function srcPorts = traceThroughOutport(subsystem, portNum)
    srcPorts = {};
    outports = find_system(subsystem, 'SearchDepth', 1, 'BlockType', 'Outport');

    for i = 1:length(outports)
        if str2double(get_param(outports{i}, 'Port')) == portNum
            inLine = get_param(get_param(outports{i}, 'PortHandles').Inport, 'Line');
            if inLine ~= -1 && inLine ~= 0
                srcPort = get_param(inLine, 'SrcPortHandle');
                srcBlock = get_param(srcPort, 'Parent');
                if strcmp(get_param(srcBlock, 'BlockType'), 'SubSystem')
                    nestedPortNum = get_param(srcPort, 'PortNumber');
                    nestedSrcPorts = traceThroughOutport(srcBlock, nestedPortNum);
                    srcPorts = [srcPorts nestedSrcPorts];
                else
                    srcPorts{end+1} = srcPort;
                end
            end
            break;
        end
    end
end

