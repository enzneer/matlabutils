function [sourceBlockHandles, destinationBlockHandles] = resolveSingleBlockConnectivity(handleToResolve, modelRootHandle, visitedInVirtualPath)
%RESOLVESINGLEBLOCKCONNECTIVITY Core logic to find block connections.
%   [sourceBlockHandles, destinationBlockHandles] = RESOLVESINGLEBLOCKCONNECTIVITY(handleToResolve, modelRootHandle, visitedInVirtualPath)
%   is a helper function for GETBLOCKCONNECTIVITY. It recursively
%   determines the effective source and destination blocks for handleToResolve.
%
%   INPUTS:
%       handleToResolve     - Handle of the block currently being resolved.
%       modelRootHandle     - Handle of the root model.
%       visitedInVirtualPath- (Optional) Array of handles visited in the current
%                             virtual subsystem traversal path, used for loop detection.
%
%   OUTPUTS:
%       sourceBlockHandles      - Array of structs {'Handle', H, 'Name', N} of effective source blocks.
%       destinationBlockHandles - Array of structs {'Handle', H, 'Name', N} of effective destination blocks.
%
%   This function handles:
%       - Direct connections via PortConnectivity.
%       - Recursive traversal through virtual subsystems.
%       - Resolution of Goto/From tags for effective connections.
%       - Loop detection for virtual paths.

    if nargin < 3
        visitedInVirtualPath = [];
    end

    if ismember(handleToResolve, visitedInVirtualPath)
        sourceBlockHandles = [];
        destinationBlockHandles = [];
        return;
    end

    sourceBlockInfoStructs = [];
    destinationBlockInfoStructs = [];
    portConnectivityOriginalBlock = get_param(handleToResolve, 'PortConnectivity');

    for i = 1:length(portConnectivityOriginalBlock)
        srcBlockH = portConnectivityOriginalBlock(i).SrcBlock;
        srcPortH = portConnectivityOriginalBlock(i).SrcPortHandle;
        dstPortH = portConnectivityOriginalBlock(i).DstPortHandle;
        if ishandle(srcBlockH) && srcBlockH ~= -1 && ~isempty(dstPortH) && ishandle(dstPortH)
            if get_param(get_param(dstPortH, 'Parent'), 'Handle') == handleToResolve
                actualSrcPortNum = get_param(srcPortH, 'PortNumber');
                try
                    srcBlockName = getfullname(srcBlockH);
                    sourceBlockInfoStructs = [sourceBlockInfoStructs; struct('Handle', srcBlockH, 'Name', srcBlockName, 'PortNumber', actualSrcPortNum)];
                catch E
                     warning('resolveSingleBlockConnectivity:GetFullNameFailed', 'Failed getfullname for source handle %s: %s. Skipping.', num2str(srcBlockH), E.message);
                end
            end
        end
    end

    for i = 1:length(portConnectivityOriginalBlock)
        srcPortH = portConnectivityOriginalBlock(i).SrcPortHandle;
        if ~isempty(portConnectivityOriginalBlock(i).DstBlock) && ~isempty(srcPortH) && ishandle(srcPortH)
            if get_param(get_param(srcPortH, 'Parent'), 'Handle') == handleToResolve
                dstPortHandlesArray = portConnectivityOriginalBlock(i).DstPortHandle;
                if ~iscell(dstPortHandlesArray) && ~isempty(dstPortHandlesArray)
                     dstPortHandlesArray = {dstPortHandlesArray};
                elseif isempty(dstPortHandlesArray)
                    continue;
                end
                for j = 1:length(portConnectivityOriginalBlock(i).DstBlock)
                    dstBlockH = portConnectivityOriginalBlock(i).DstBlock(j);
                    if ishandle(dstBlockH) && dstBlockH ~= -1
                        for k_port = 1:length(dstPortHandlesArray)
                            currentDstPortH = dstPortHandlesArray{k_port};
                            if ishandle(currentDstPortH) && get_param(get_param(currentDstPortH, 'Parent'),'Handle') == dstBlockH
                                destPortNum = get_param(currentDstPortH, 'PortNumber');
                                try
                                    dstBlockName = getfullname(dstBlockH);
                                    destinationBlockInfoStructs = [destinationBlockInfoStructs; struct('Handle', dstBlockH, 'Name', dstBlockName, 'PortNumber', destPortNum)];
                                catch E
                                    warning('resolveSingleBlockConnectivity:GetFullNameFailed', 'Failed getfullname for dest handle %s: %s. Skipping.', num2str(dstBlockH), E.message);
                                end
                                % Removed break; to capture all connections to the same destination block
                            end
                        end
                    end
                end
            end
        end
    end

    % VIRTUAL SUBSYSTEM LOGIC - Refactored
    effectiveSourceStructs = [];
    for k = 1:length(sourceBlockInfoStructs)
        currentSrcInfo = sourceBlockInfoStructs(k);
        processedStructs = process_virtual_connection(currentSrcInfo, 'source', modelRootHandle, visitedInVirtualPath, handleToResolve);
        effectiveSourceStructs = [effectiveSourceStructs; processedStructs];
    end
    sourceBlockHandles = effectiveSourceStructs;

    effectiveDestinationStructs = [];
    for k = 1:length(destinationBlockInfoStructs)
        currentDestInfo = destinationBlockInfoStructs(k);
        processedStructs = process_virtual_connection(currentDestInfo, 'destination', modelRootHandle, visitedInVirtualPath, handleToResolve);
        effectiveDestinationStructs = [effectiveDestinationStructs; processedStructs];
    end
    destinationBlockHandles = effectiveDestinationStructs;
    % --- END VIRTUAL SUBSYSTEM LOGIC ---

    originalBlockType = get_param(handleToResolve, 'BlockType');
    originalBlockFullName = getfullname(handleToResolve);

    tempSourceStructs = [];
    for i = 1:length(sourceBlockHandles)
        srcInfo = sourceBlockHandles(i);
        if ~ishandle(srcInfo.Handle)
            warning('resolveSingleBlockConnectivity:InvalidHandleInList', 'Invalid handle for %s. Skipping.', srcInfo.Name);
            continue;
        end
        try
            srcBlockType = get_param(srcInfo.Handle, 'BlockType');
            if strcmp(srcBlockType, 'From')
                fromTag = get_param(srcInfo.Handle, 'GotoTag');
                % PERFORMANCE NOTE: For very large models and frequent calls to getBlockConnectivity,
                % consider implementing a caching mechanism for these find_system results based on
                % modelRootHandle, GotoTag, and BlockType to avoid redundant model-wide searches.
                gotos = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'Goto', 'GotoTag', fromTag);
                for idx = 1:length(gotos)
                    if ishandle(gotos(idx))
                        tempSourceStructs = [tempSourceStructs; struct('Handle', gotos(idx), 'Name', getfullname(gotos(idx)))];
                    end
                end
            else
                tempSourceStructs = [tempSourceStructs; srcInfo];
            end
        catch E
            warning('resolveSingleBlockConnectivity:GetParamFailedFromProc', 'Failed for source %s: %s. Keeping original.', srcInfo.Name, E.message);
            tempSourceStructs = [tempSourceStructs; srcInfo];
        end
    end
    sourceBlockHandles = tempSourceStructs;

    tempDestinationStructs = [];
    for i = 1:length(destinationBlockHandles)
        destInfo = destinationBlockHandles(i);
        if ~ishandle(destInfo.Handle)
            warning('resolveSingleBlockConnectivity:InvalidHandleInList', 'Invalid handle for %s. Skipping.', destInfo.Name);
            continue;
        end
        try
            destBlockType = get_param(destInfo.Handle, 'BlockType');
            if strcmp(destBlockType, 'Goto')
                gotoTag = get_param(destInfo.Handle, 'GotoTag');
                % PERFORMANCE NOTE: For very large models and frequent calls to getBlockConnectivity,
                % consider implementing a caching mechanism for these find_system results based on
                % modelRootHandle, GotoTag, and BlockType to avoid redundant model-wide searches.
                froms = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'From', 'GotoTag', gotoTag);
                 for idx = 1:length(froms)
                    if ishandle(froms(idx))
                        tempDestinationStructs = [tempDestinationStructs; struct('Handle', froms(idx), 'Name', getfullname(froms(idx)))];
                    end
                end
            else
                tempDestinationStructs = [tempDestinationStructs; destInfo];
            end
        catch E
            warning('resolveSingleBlockConnectivity:GetParamFailedGotoProc', 'Failed for dest %s: %s. Keeping original.', destInfo.Name, E.message);
            tempDestinationStructs = [tempDestinationStructs; destInfo];
        end
    end
    destinationBlockHandles = tempDestinationStructs;

    try
        if strcmp(originalBlockType, 'Goto')
            gotoTag = get_param(handleToResolve, 'GotoTag');
            % PERFORMANCE NOTE: For very large models and frequent calls to getBlockConnectivity,
            % consider implementing a caching mechanism for these find_system results based on
            % modelRootHandle, GotoTag, and BlockType to avoid redundant model-wide searches.
            froms = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'From', 'GotoTag', gotoTag);
            for idx = 1:length(froms)
                if ishandle(froms(idx))
                    destinationBlockHandles = [destinationBlockHandles; struct('Handle', froms(idx), 'Name', getfullname(froms(idx)))];
                end
            end
        elseif strcmp(originalBlockType, 'From')
            fromTag = get_param(handleToResolve, 'GotoTag');
            % PERFORMANCE NOTE: For very large models and frequent calls to getBlockConnectivity,
            % consider implementing a caching mechanism for these find_system results based on
            % modelRootHandle, GotoTag, and BlockType to avoid redundant model-wide searches.
            gotos = find_system(modelRootHandle, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'MatchFilter', @Simulink.match.allVariants, 'BlockType', 'Goto', 'GotoTag', fromTag);
            for idx = 1:length(gotos)
                if ishandle(gotos(idx))
                    sourceBlockHandles = [sourceBlockHandles; struct('Handle', gotos(idx), 'Name', getfullname(gotos(idx)))];
                end
            end
        end
    catch E
        warning('resolveSingleBlockConnectivity:GetParamFailedMainGotoFrom', 'Failed for %s: %s.', originalBlockFullName, E.message);
    end
end

% --- Local Helper Function for Virtual Subsystem Processing ---
function effectiveStructs = process_virtual_connection(connectedBlockInfo, connectionType, modelRootHandle, visitedInVirtualPath, originalHandleToResolve)
    effectiveStructs = [];
    isVirtual = false;

    if ishandle(connectedBlockInfo.Handle)
        try
            bt = get_param(connectedBlockInfo.Handle, 'BlockType');
            ot = get_param(connectedBlockInfo.Handle, 'Type');
            if strcmp(ot, 'block') && strcmp(bt, 'SubSystem') && strcmp(get_param(connectedBlockInfo.Handle, 'IsSubsystemVirtual'), 'on')
                isVirtual = true;
            end
        catch E
            warning('resolveSingleBlockConnectivity:ProcessVirtual:GetParamFailed', ...
                    'Failed virtual check for %s: %s. Assuming non-virtual.', connectedBlockInfo.Name, E.message);
        end
    end

    if isVirtual
        matchFound = false;
        internalPortBlocks = [];
        if strcmp(connectionType, 'source') % Virtual source: look for Outports inside it
            internalPortBlocks = find_system(connectedBlockInfo.Handle, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Outport');
        elseif strcmp(connectionType, 'destination') % Virtual destination: look for Inports inside it
            internalPortBlocks = find_system(connectedBlockInfo.Handle, 'SearchDepth', 1, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'BlockType', 'Inport');
        end

        for l = 1:length(internalPortBlocks)
            portBlockH = internalPortBlocks(l);
            if ~ishandle(portBlockH) continue; end
            try
                if strcmp(get_param(portBlockH, 'Port'), num2str(connectedBlockInfo.PortNumber))
                    newVisitedPath = [visitedInVirtualPath, originalHandleToResolve];
                    if strcmp(connectionType, 'source')
                        [internalSources, ~] = resolveSingleBlockConnectivity(portBlockH, modelRootHandle, newVisitedPath);
                        effectiveStructs = [effectiveStructs; internalSources];
                    elseif strcmp(connectionType, 'destination')
                        [~, internalDests] = resolveSingleBlockConnectivity(portBlockH, modelRootHandle, newVisitedPath);
                        effectiveStructs = [effectiveStructs; internalDests];
                    end
                    matchFound = true;
                end
            catch E
                 warning('resolveSingleBlockConnectivity:ProcessVirtual:GetPortFailed', ...
                         'Failed to get Port for internal block %s: %s.', getfullname(portBlockH), E.message);
            end
        end
        if ~matchFound
             effectiveStructs = [effectiveStructs; struct('Handle', connectedBlockInfo.Handle, 'Name', connectedBlockInfo.Name)]; % Fallback
        end
    else
        effectiveStructs = [effectiveStructs; struct('Handle', connectedBlockInfo.Handle, 'Name', connectedBlockInfo.Name)];
    end
end
