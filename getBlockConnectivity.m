function connectivityInfo = getBlockConnectivity(blockHandle)
    % Initializes the output structure
    connectivityInfo = struct('sourceBlocks', {{}}, 'destinationBlocks', {{}});

    % Check if the handle is a valid Simulink block
    if ~ishandle(blockHandle) || ~strcmp(get_param(blockHandle, 'Type'), 'block')
        disp(['Error: Input is not a valid Simulink block handle. Provided value: ', num2str(blockHandle)]);
        return; % Return the empty connectivityInfo
    end

    blockType = get_param(blockHandle, 'BlockType');
    portConnectivity = get_param(blockHandle, 'PortConnectivity');

    currentBlockName = getfullname(blockHandle); % Get full path for better context in debugging

    sourceBlockHandles = [];
    destinationBlockHandles = [];

    % Get source blocks (inputs to current block)
    for i = 1:length(portConnectivity)
        if ishandle(portConnectivity(i).SrcBlock) && portConnectivity(i).SrcBlock ~= -1
            % Check if connection is to an input port of the current block
            % Ensure DstPortHandle is valid and its parent is the blockHandle
            if ~isempty(portConnectivity(i).DstPortHandle) && ishandle(portConnectivity(i).DstPortHandle)
                portParentHandle = get_param(get_param(portConnectivity(i).DstPortHandle, 'Parent'), 'Handle');
                if portParentHandle == blockHandle
                    sourceBlockHandles = [sourceBlockHandles; portConnectivity(i).SrcBlock];
                end
            end
        end
    end

    % Get destination blocks (outputs from current block)
    for i = 1:length(portConnectivity)
        if ~isempty(portConnectivity(i).DstBlock)
             % Check if connection is from an output port of the current block
             % Ensure SrcPortHandle is valid and its parent is the blockHandle
            if ~isempty(portConnectivity(i).SrcPortHandle) && ishandle(portConnectivity(i).SrcPortHandle)
                portParentHandle = get_param(get_param(portConnectivity(i).SrcPortHandle, 'Parent'), 'Handle');
                if portParentHandle == blockHandle
                    for j = 1:length(portConnectivity(i).DstBlock)
                        if ishandle(portConnectivity(i).DstBlock(j)) && portConnectivity(i).DstBlock(j) ~= -1
                            destinationBlockHandles = [destinationBlockHandles; portConnectivity(i).DstBlock(j)];
                        end
                    end
                end
            end
        end
    end

    % Initial population of connectivityInfo
    connectivityInfo.sourceBlocks = unique(sourceBlockHandles);
    connectivityInfo.destinationBlocks = unique(destinationBlockHandles);

    % Handle specific block types
    if strcmp(blockType, 'Goto')
        gotoTag = get_param(blockHandle, 'GotoTag');
        fromBlocksInModel = find_system(bdroot(blockHandle), ...
                                        'LookUnderMasks', 'all', ...
                                        'FollowLinks', 'on', ...
                                        'MatchFilter', @Simulink.match.allVariants, ... % Important for variants
                                        'BlockType', 'From', ...
                                        'GotoTag', gotoTag);

        % Add these to the destination blocks, ensuring uniqueness
        % Convert to column vector before concatenating if not empty
        if ~isempty(fromBlocksInModel)
            connectivityInfo.destinationBlocks = unique([connectivityInfo.destinationBlocks; fromBlocksInModel(:)]);
        end

    elseif strcmp(blockType, 'From')
        fromTag = get_param(blockHandle, 'GotoTag'); % For 'From' blocks, it's also 'GotoTag'
        % Find all Goto blocks in the model that match this tag
        % Typically, there should be only one Goto block for a given tag.
        % If multiple exist, this will find all of them.
        gotoBlocksInModel = find_system(bdroot(blockHandle), ...
                                        'LookUnderMasks', 'all', ...
                                        'FollowLinks', 'on', ...
                                        'MatchFilter', @Simulink.match.allVariants, ... % Important for variants
                                        'BlockType', 'Goto', ...
                                        'GotoTag', fromTag);

        % Add these to the source blocks, ensuring uniqueness
        % Convert to column vector before concatenating if not empty
        if ~isempty(gotoBlocksInModel)
            connectivityInfo.sourceBlocks = unique([connectivityInfo.sourceBlocks; gotoBlocksInModel(:)]);
        end
    end

    % Ensure cell arrays for output
    if ~iscell(connectivityInfo.sourceBlocks)
        connectivityInfo.sourceBlocks = num2cell(connectivityInfo.sourceBlocks);
    end
    if ~iscell(connectivityInfo.destinationBlocks)
        connectivityInfo.destinationBlocks = num2cell(connectivityInfo.destinationBlocks);
    end

    % Remove empty cells if any were created by num2cell on an empty matrix,
    % or if initial arrays were empty.
    if numel(connectivityInfo.sourceBlocks) == 1 && isempty(connectivityInfo.sourceBlocks{1})
        connectivityInfo.sourceBlocks = {}; % Use {} for an empty cell array
    elseif ~isempty(connectivityInfo.sourceBlocks)
        connectivityInfo.sourceBlocks = connectivityInfo.sourceBlocks(~cellfun('isempty', connectivityInfo.sourceBlocks));
    else
        connectivityInfo.sourceBlocks = {}; % Ensure it's an empty cell if it started empty
    end

    if numel(connectivityInfo.destinationBlocks) == 1 && isempty(connectivityInfo.destinationBlocks{1})
        connectivityInfo.destinationBlocks = {}; % Use {} for an empty cell array
    elseif ~isempty(connectivityInfo.destinationBlocks)
        connectivityInfo.destinationBlocks = connectivityInfo.destinationBlocks(~cellfun('isempty', connectivityInfo.destinationBlocks));
    else
        connectivityInfo.destinationBlocks = {}; % Ensure it's an empty cell if it started empty
    end
end
