function connectivityInfo = getBlockConnectivity(blockHandle)
    % Initializes the output structure
    connectivityInfo = struct('sourceBlocks', {{}}, 'destinationBlocks', {{}});

    % Check if the handle is a valid Simulink block
    if ~ishandle(blockHandle) || ~strcmp(get_param(blockHandle, 'Type'), 'block')
        disp(['Error: Input is not a valid Simulink block handle. Provided value: ', num2str(blockHandle)]);
        return; % Return the empty connectivityInfo
    end

    modelRootHandle = bdroot(blockHandle); % Get model root once

    % Call the helper function to do the main work
    [sourceHandles, destinationHandles] = resolveSingleBlockConnectivity(blockHandle, modelRootHandle);

    % Ensure uniqueness and cell array format for final output
    connectivityInfo.sourceBlocks = unique(sourceHandles);
    connectivityInfo.destinationBlocks = unique(destinationHandles);

    if ~iscell(connectivityInfo.sourceBlocks)
        connectivityInfo.sourceBlocks = num2cell(connectivityInfo.sourceBlocks);
    end
    if ~iscell(connectivityInfo.destinationBlocks)
        connectivityInfo.destinationBlocks = num2cell(connectivityInfo.destinationBlocks);
    end

    if numel(connectivityInfo.sourceBlocks) == 1 && isempty(connectivityInfo.sourceBlocks{1})
        connectivityInfo.sourceBlocks = {};
    elseif ~isempty(connectivityInfo.sourceBlocks)
        connectivityInfo.sourceBlocks = connectivityInfo.sourceBlocks(~cellfun('isempty', connectivityInfo.sourceBlocks));
    else
        connectivityInfo.sourceBlocks = {};
    end

    if numel(connectivityInfo.destinationBlocks) == 1 && isempty(connectivityInfo.destinationBlocks{1})
        connectivityInfo.destinationBlocks = {};
    elseif ~isempty(connectivityInfo.destinationBlocks)
        connectivityInfo.destinationBlocks = connectivityInfo.destinationBlocks(~cellfun('isempty', connectivityInfo.destinationBlocks));
    else
        connectivityInfo.destinationBlocks = {};
    end
end

% Helper function to resolve connectivity for a single block
function [sourceBlockHandles, destinationBlockHandles] = resolveSingleBlockConnectivity(handleToResolve, modelRootHandle, visitedInVirtualPath)
    if nargin < 3
        visitedInVirtualPath = [];
    end

    % Loop detection for virtual subsystem traversal
    if ismember(handleToResolve, visitedInVirtualPath)
        % disp(['Warning: Loop detected in virtual subsystem traversal path for block: ', getfullname(handleToResolve)]);
        sourceBlockHandles = [];
        destinationBlockHandles = [];
        return;
    end

    % Initialize outputs for this resolver
    sourceBlockHandlesStruct = []; % Will store structs {Handle, PortNumberOnThatHandle}
    destinationBlockHandlesStruct = []; % Will store structs {Handle, PortNumberOnThatHandle}

    portConnectivityOriginalBlock = get_param(handleToResolve, 'PortConnectivity');

    % Get initial source blocks (inputs to handleToResolve)
    for i = 1:length(portConnectivityOriginalBlock)
        srcBlock = portConnectivityOriginalBlock(i).SrcBlock;
        srcPortHandle = portConnectivityOriginalBlock(i).SrcPortHandle; % Port handle on the source block
        dstPortHandle = portConnectivityOriginalBlock(i).DstPortHandle; % Port handle on handleToResolve

        if ishandle(srcBlock) && srcBlock ~= -1 && ~isempty(dstPortHandle) && ishandle(dstPortHandle)
            portParentHandle = get_param(get_param(dstPortHandle, 'Parent'), 'Handle');
            if portParentHandle == handleToResolve
                % PortNumber is the output port number of the SrcBlock
                actualSrcPortNumOnSrcBlock = get_param(srcPortHandle, 'PortNumber');
                sourceBlockHandlesStruct = [sourceBlockHandlesStruct; struct('Handle', srcBlock, 'PortNumber', actualSrcPortNumOnSrcBlock)];
            end
        end
    end

    % Get initial destination blocks (outputs from handleToResolve)
    for i = 1:length(portConnectivityOriginalBlock)
        srcPortHandle = portConnectivityOriginalBlock(i).SrcPortHandle; % Port handle on handleToResolve

        if ~isempty(portConnectivityOriginalBlock(i).DstBlock) && ~isempty(srcPortHandle) && ishandle(srcPortHandle)
            portParentHandle = get_param(get_param(srcPortHandle, 'Parent'), 'Handle');
            if portParentHandle == handleToResolve % Connection is from an output port of handleToResolve

                dstPortHandlesArray = portConnectivityOriginalBlock(i).DstPortHandle; % These are ports on destination blocks
                if ~iscell(dstPortHandlesArray) && ~isempty(dstPortHandlesArray) % Ensure it's an array of handles
                     dstPortHandlesArray = {dstPortHandlesArray}; % Make it a cell for iteration if single handle
                elseif isempty(dstPortHandlesArray)
                    continue; % No destination ports, skip
                end

                for j = 1:length(portConnectivityOriginalBlock(i).DstBlock)
                    dstBlock = portConnectivityOriginalBlock(i).DstBlock(j);
                    if ishandle(dstBlock) && dstBlock ~= -1
                        % Find the specific DstPortHandle on the DstBlock that matches this connection
                        foundPortMatchForDest = false;
                        for k_port = 1:length(dstPortHandlesArray)
                            currentDstPortHandle = dstPortHandlesArray{k_port};
                            if ishandle(currentDstPortHandle) && get_param(get_param(currentDstPortHandle, 'Parent'),'Handle') == dstBlock
                                destPortNumOnDstBlock = get_param(currentDstPortHandle, 'PortNumber');
                                destinationBlockHandlesStruct = [destinationBlockHandlesStruct; struct('Handle', dstBlock, 'PortNumber', destPortNumOnDstBlock)];
                                foundPortMatchForDest = true;
                                % Assuming one-to-one connection for a given DstBlock entry for simplicity of port matching here
                                % If a single output port of handleToResolve connects to multiple input ports of the SAME DstBlock,
                                % this would only capture the first match. This is rare.
                                break;
                            end
                        end
                    end
                end
            end
        end
    end

    % --- VIRTUAL SUBSYSTEM LOGIC ---
    effectiveSourceHandles = [];
    for k = 1:length(sourceBlockHandlesStruct)
        currentSrcStruct = sourceBlockHandlesStruct(k);
        isVirtual = false;
        try
            if strcmp(get_param(currentSrcStruct.Handle, 'Type'), 'block') && strcmp(get_param(currentSrcStruct.Handle, 'BlockType'), 'SubSystem')
                if strcmp(get_param(currentSrcStruct.Handle, 'IsSubsystemVirtual'), 'on')
                    isVirtual = true;
                end
            end
        catch E
            % Not a subsystem or parameter doesn't exist, so not virtual
        end

        if isVirtual
            % Source is a virtual subsystem. We need to find the Outport block inside it
            % that corresponds to currentSrcStruct.PortNumber (which is the port on the virtual subsystem itself).
            outportBlocks = find_system(currentSrcStruct.Handle, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Outport');
            foundMatch = false;
            for l = 1:length(outportBlocks)
                if strcmp(get_param(outportBlocks(l), 'Port'), num2str(currentSrcStruct.PortNumber))
                    newVisited = [visitedInVirtualPath, handleToResolve]; % Add current block before diving deeper
                    % Recursively resolve what's connected to this Outport's input
                    [internalSources, ~] = resolveSingleBlockConnectivity(outportBlocks(l), modelRootHandle, newVisited);
                    effectiveSourceHandles = [effectiveSourceHandles; internalSources];
                    foundMatch = true;
                    % Assuming one Outport block matches the port number. If multiple, takes first.
                    % If an Outport has multiple internal sources, all will be added.
                end
            end
            if ~foundMatch
                 effectiveSourceHandles = [effectiveSourceHandles; currentSrcStruct.Handle]; % Fallback to subsystem itself
            end
        else
            effectiveSourceHandles = [effectiveSourceHandles; currentSrcStruct.Handle];
        end
    end
    sourceBlockHandles = unique(effectiveSourceHandles);

    effectiveDestinationHandles = [];
    for k = 1:length(destinationBlockHandlesStruct)
        currentDestStruct = destinationBlockHandlesStruct(k);
        isVirtual = false;
        try
            if strcmp(get_param(currentDestStruct.Handle, 'Type'), 'block') && strcmp(get_param(currentDestStruct.Handle, 'BlockType'), 'SubSystem')
                if strcmp(get_param(currentDestStruct.Handle, 'IsSubsystemVirtual'), 'on')
                    isVirtual = true;
                end
            end
        catch E
            % Not a subsystem or parameter doesn't exist
        end

        if isVirtual
            % Destination is a virtual subsystem. We need to find the Inport block inside it
            % that corresponds to currentDestStruct.PortNumber.
            inportBlocks = find_system(currentDestStruct.Handle, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Inport');
            foundMatch = false;
            for l = 1:length(inportBlocks)
                if strcmp(get_param(inportBlocks(l), 'Port'), num2str(currentDestStruct.PortNumber))
                    newVisited = [visitedInVirtualPath, handleToResolve]; % Add current block
                    % Recursively resolve what this Inport's output connects to
                    [~, internalDestinations] = resolveSingleBlockConnectivity(inportBlocks(l), modelRootHandle, newVisited);
                    effectiveDestinationHandles = [effectiveDestinationHandles; internalDestinations];
                    foundMatch = true;
                    % Assuming one Inport block matches the port number.
                    % If an Inport has multiple internal destinations, all will be added.
                end
            end
            if ~foundMatch
                effectiveDestinationHandles = [effectiveDestinationHandles; currentDestStruct.Handle]; % Fallback
            end
        else
            effectiveDestinationHandles = [effectiveDestinationHandles; currentDestStruct.Handle];
        end
    end
    destinationBlockHandles = unique(effectiveDestinationHandles);
    % --- END VIRTUAL SUBSYSTEM LOGIC ---

    % Store original block type for Goto/From logic related to handleToResolve itself
    originalBlockType = get_param(handleToResolve, 'BlockType');

    % Option B: If an *effective* source/destination (after virtual passthrough) is a Goto/From.
    % This needs to run before the logic for handleToResolve itself being a Goto/From,
    % to correctly trace through chains like Signal -> From (becomes Goto) -> original block.

    tempSourceHandles = [];
    for i = 1:length(sourceBlockHandles)
        srcH = sourceBlockHandles(i);
        srcBlockType = get_param(srcH, 'BlockType');
        if strcmp(srcBlockType, 'From')
            fromTag = get_param(srcH, 'GotoTag');
            gotoBlocks = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'Goto', 'GotoTag', fromTag);
            tempSourceHandles = [tempSourceHandles; gotoBlocks(:)];
        else
            tempSourceHandles = [tempSourceHandles; srcH];
        end
    end
    sourceBlockHandles = unique(tempSourceHandles);

    tempDestinationHandles = [];
    for i = 1:length(destinationBlockHandles)
        destH = destinationBlockHandles(i);
        destBlockType = get_param(destH, 'BlockType');
        if strcmp(destBlockType, 'Goto')
            gotoTag = get_param(destH, 'GotoTag');
            fromBlocks = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'From', 'GotoTag', gotoTag);
            tempDestinationHandles = [tempDestinationHandles; fromBlocks(:)];
        else
            tempDestinationHandles = [tempDestinationHandles; destH];
        end
    end
    destinationBlockHandles = unique(tempDestinationHandles);

    % Original Goto/From logic for handleToResolve itself
    if strcmp(originalBlockType, 'Goto')
        gotoTag = get_param(handleToResolve, 'GotoTag');
        fromBlocksInModel = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'From', 'GotoTag', gotoTag);
        if ~isempty(fromBlocksInModel)
            % For a Goto, its destinations are the From blocks.
            % Its sources are whatever feeds into it (already handled by port connectivity & virtual logic).
            destinationBlockHandles = unique([destinationBlockHandles; fromBlocksInModel(:)]);
        end

    elseif strcmp(originalBlockType, 'From')
        fromTag = get_param(handleToResolve, 'GotoTag');
        gotoBlocksInModel = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'Goto', 'GotoTag', fromTag);
        if ~isempty(gotoBlocksInModel)
            % For a From, its sources are the Goto blocks.
            % Its destinations are whatever it feeds (already handled by port connectivity & virtual logic).
            sourceBlockHandles = unique([sourceBlockHandles; gotoBlocksInModel(:)]);
        end
    end

    % Final unique results before returning
    sourceBlockHandles = unique(sourceBlockHandles);
    destinationBlockHandles = unique(destinationBlockHandles);
end
