function connectivityInfo = getBlockConnectivity(blockHandle)
%GETBLOCKCONNECTIVITY Identifies source and destination blocks for a Simulink block.
%   connectivityInfo = GETBLOCKCONNECTIVITY(blockHandle) returns a struct
%   containing the handles and names of blocks that are sources to or
%   destinations from the block specified by blockHandle.
%
%   INPUT:
%       blockHandle - Handle of the Simulink block to analyze.
%
%   OUTPUT:
%       connectivityInfo - A struct with two fields:
%           .sourceBlocks      - Cell array of structs {'Handle', H, 'Name', N}.
%           .destinationBlocks - Cell array of structs {'Handle', H, 'Name', N}.
%
%   BEHAVIOR:
%       - Traverses virtual subsystems to find true underlying connections.
%       - Resolves Goto/From block links.
%       - Includes loop detection for virtual paths.
%
%   See also RESOLVESINGLEBLOCKCONNECTIVITY.

    connectivityInfo = struct('sourceBlocks', {{}}, 'destinationBlocks', {{}});

    if ~ishandle(blockHandle)
        warning('getBlockConnectivity:InvalidHandle', 'Input is not a valid handle.');
        return;
    end
    try
        if ~strcmp(get_param(blockHandle, 'Type'), 'block')
            warning('getBlockConnectivity:NotABlock', 'Input handle is not a block.');
            return;
        end
    catch E
        warning('getBlockConnectivity:GetParamError', 'Error calling get_param on input handle: %s', E.message);
        return;
    end

    modelRootHandle = bdroot(blockHandle);

    % Returns arrays of structs: struct('Handle', H, 'Name', N)
    [sourceStructArray, destinationStructArray] = resolveSingleBlockConnectivity(blockHandle, modelRootHandle);

    % Uniqueness based on Handle field
    if ~isempty(sourceStructArray)
        [~, uniqueIdx] = unique([sourceStructArray.Handle], 'stable');
        connectivityInfo.sourceBlocks = sourceStructArray(uniqueIdx);
    else
        % Ensure it's an empty struct array with correct fields if completely empty,
        % so num2cell doesn't error and results in {}
        connectivityInfo.sourceBlocks = repmat(struct('Handle',[],'Name',[]), 0, 1);
    end

    if ~isempty(destinationStructArray)
        [~, uniqueIdx] = unique([destinationStructArray.Handle], 'stable');
        connectivityInfo.destinationBlocks = destinationStructArray(uniqueIdx);
    else
        connectivityInfo.destinationBlocks = repmat(struct('Handle',[],'Name',[]), 0, 1);
    end

    % Convert array of structs to cell array of structs
    % num2cell will create a 1xN cell array if the input is a 1xN struct array.
    % If input is an empty struct array (0x1 or 0x0), num2cell returns an empty cell array {}.
    if ~isempty(connectivityInfo.sourceBlocks) || isstruct(connectivityInfo.sourceBlocks)
        % isstruct check is to ensure repmat output (empty struct array) is also processed
        connectivityInfo.sourceBlocks = num2cell(connectivityInfo.sourceBlocks);
    else
        connectivityInfo.sourceBlocks = {}; % Should be already if empty struct array was passed to num2cell
    end

    if ~isempty(connectivityInfo.destinationBlocks) || isstruct(connectivityInfo.destinationBlocks)
        connectivityInfo.destinationBlocks = num2cell(connectivityInfo.destinationBlocks);
    else
        connectivityInfo.destinationBlocks = {};
    end

    % Ensure truly empty cell {} if it started empty and num2cell might produce {{[]}} or similar
    if iscell(connectivityInfo.sourceBlocks) && numel(connectivityInfo.sourceBlocks) == 1 && ...
       isstruct(connectivityInfo.sourceBlocks{1}) && isempty(fieldnames(connectivityInfo.sourceBlocks{1}))
        connectivityInfo.sourceBlocks = {};
    end
    if iscell(connectivityInfo.destinationBlocks) && numel(connectivityInfo.destinationBlocks) == 1 && ...
       isstruct(connectivityInfo.destinationBlocks{1}) && isempty(fieldnames(connectivityInfo.destinationBlocks{1}))
        connectivityInfo.destinationBlocks = {};
    end

end
