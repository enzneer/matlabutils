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
    numDataPorts = 0;

    % Handle data input ports
    if isfield(ports, 'Inport')
        numDataPorts = length(ports.Inport);
        for p = 1:numDataPorts
            inNodeName = sprintf('%s_in%d', blockName, p);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, inNodeName, blockName, p, 'input', nextId);

            edgeList = addEdge(edgeList, nodeMap(inNodeName), nodeMap(blockName));

            line = get_param(ports.Inport(p), 'Line');
            [nodeMap, nodeList, edgeList, nextId] = handleInputLineSource( ...
                inNodeName, line, nodeMap, nodeList, edgeList, nextId);
        end
    end

    % Handle control ports (Enable, Trigger, Action)
    controlTypes = {'Enable', 'Trigger', 'Action'};
    for ct = 1:length(controlTypes)
        type = controlTypes{ct};
        if isfield(ports, type)
            controlHandles = ports.(type);
            if ~iscell(controlHandles)
                controlHandles = num2cell(controlHandles);
            end
            for c = 1:length(controlHandles)
                portHandle = controlHandles{c};
                portNumber = numDataPorts + c;
                inNodeName = sprintf('%s_%s%d', blockName, lower(type), c);

                [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                    nodeMap, nodeList, inNodeName, blockName, portNumber, 'input', nextId);

                edgeList = addEdge(edgeList, nodeMap(inNodeName), nodeMap(blockName));

                line = get_param(portHandle, 'Line');
                [nodeMap, nodeList, edgeList, nextId] = handleInputLineSource( ...
                    inNodeName, line, nodeMap, nodeList, edgeList, nextId);
            end
        end
    end
end


function [nodeMap, nodeList, edgeList, nextId] = handleInputLineSource(inNodeName, line, nodeMap, nodeList, edgeList, nextId)
    if line ~= -1 && line ~= 0
        srcPort = get_param(line, 'SrcPortHandle');
        srcBlock = get_param(srcPort, 'Parent');
        srcBlockType = get_param(srcBlock, 'BlockType');

        if strcmp(srcBlockType, 'SubSystem')
            % Trace through subsystem's Outport
            portNum = get_param(srcPort, 'PortNumber');
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

        elseif strcmp(srcBlockType, 'Outport')
            % Skip edge from Outport block itself
            % Instead trace back to the actual source inside the subsystem
            portNum = get_param(srcPort, 'PortNumber');
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

        elseif ~ismember(srcBlockType, {'Inport'})
            srcBlockName = getfullname(srcBlock);
            srcPortNum = get_param(srcPort, 'PortNumber');
            outNodeName = sprintf('%s_out%d', srcBlockName, srcPortNum);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, outNodeName, srcBlockName, srcPortNum, 'output', nextId);
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
    if isfield(ports, 'Outport')
        for p = 1:length(ports.Outport)
            outNodeName = sprintf('%s_out%d', blockName, p);

            % Create output port node if needed
            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, outNodeName, blockName, p, 'output', nextId);

            % Add edge from block to output port
            edgeList = addEdge(edgeList, nodeMap(blockName), nodeMap(outNodeName));

            % Handle destinations of the output line
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

        if strcmp(dstBlockType, 'SubSystem')
            portNum = get_param(dstPorts{j}, 'PortNumber');
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

        elseif ismember(dstBlockType, {'Inport', 'Enable', 'Trigger', 'Action'})
            % Skip edge to subsystem port block
            % Instead, trace through to the actual destination
            portNum = get_param(dstPorts{j}, 'PortNumber');
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
            dstPortNum = get_param(dstPorts{j}, 'PortNumber');
            dstNodeName = sprintf('%s_in%d', dstBlockName, dstPortNum);

            [nodeMap, nodeList, nextId] = createPortNodeIfNeeded( ...
                nodeMap, nodeList, dstNodeName, dstBlockName, dstPortNum, 'input', nextId);
            edgeList = addEdge(edgeList, nodeMap(outNodeName), nodeMap(dstNodeName));
        end
    end
end


function graph = finalizeGraph(nodeList, edgeList)
    graph = struct();
    graph.Nodes = nodeList;
    graph.Edges = edgeList;
end

function writeGraphToJSON(graph, modelName)
    jsonStr = jsonencode(graph);
    jsonStr = prettyPrintJSON(jsonStr);
    fileName = [modelName '_graph.json'];
    fid = fopen(fileName, 'w');
    if fid == -1
        error('Cannot create JSON file: %s', fileName);
    end
    fprintf(fid, '%s', jsonStr);
    fclose(fid);
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
            outLine = get_param(get_param(inports{i}, 'PortHandles').Outport, 'Line');
            if outLine ~= -1 && outLine ~= 0
                nextDstPorts = get_param(outLine, 'DstPortHandle');
                if ~iscell(nextDstPorts)
                    nextDstPorts = {nextDstPorts};
                end
                for j = 1:length(nextDstPorts)
                    dstBlock = get_param(nextDstPorts{j}, 'Parent');
                    if strcmp(get_param(dstBlock, 'BlockType'), 'SubSystem')
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


