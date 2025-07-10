function y=connectTrees(RRT1, RRT2, nodeA_index, nodeB_index)
% RRT1 and RRT2 are the two trees
% nodeA_index: Index of the node in RRT1
% nodeB_index: Index of the node in RRT2 (to be the new parent for nodeA)

% Step 1: Get the node from RRT1 that needs to be connected
nodeA = RRT1.nodes(nodeA_index);

% Step 2: Update the parent of nodeA to point to nodeB in RRT2
nodeB = RRT2.nodes(nodeB_index);

% Set the parent index of nodeA to nodeB's index in RRT2
nodeA.parent_index = nodeB_index;  % Or use a different indexing system if needed

% Step 3: Update nodeA's path properties (e.g., distance2start) based on the new tree
% We assume RRT1 and RRT2 both have nodes with 'distance2start' properties
nodeA.distance2start = nodeB.distance2start + norm(nodeA.position - nodeB.position);

% (Optional) Update any other properties that rely on the tree structure
% For example, you may want to propagate changes to other nodes connected to nodeA
updateChildrenNodes(RRT1, nodeA_index);
y=[];
end