class Node:
    def __init__(self, key, edge):
        self.key = key
        self.edge = edge
        self.left = None
        self.right = None
        self.height = 1

    def __str__(self) -> str:
        return f"Node(angle:{self.key[0]}, distance:{self.key[1]}, edge:{self.edge})"


class BalancedBinarySearchTree:
    def __init__(self):
        self.root = None

    def insert(self, key, edge):
        self.root = self._insert_helper(self.root, key, edge)

    def _insert_helper(self, node: Node, key, edge):

        if node is None:
            return Node(key, edge)
        elif key < node.key:
            node.left = self._insert_helper(node.left, key, edge)
        else:
            node.right = self._insert_helper(node.right, key, edge)

        node.height = 1 + max(self._get_height(node.left), self._get_height(node.right))

        # Update the balance factor and balance the tree
        balanceFactor = self._get_balance_factor(node)
        if balanceFactor > 1:
            if key <= node.left.key:
                return self._right_rotate(node)
            else:
                node.left = self._left_rotate(node.left)
                return self._right_rotate(node)

        if balanceFactor < -1:
            if key >= node.right.key:
                return self._left_rotate(node)
            else:
                node.right = self._right_rotate(node.right)
                return self._left_rotate(node)

        # if balance_factor > 1 and (angle, distance) < (node.left.angle, node.left.distance):
        #     return self._right_rotate(node)

        # if balance_factor > 1 and (angle, distance) > (node.left.angle, node.left.distance):
        #     node.left = self._left_rotate(node.left)
        #     return self._right_rotate(node)

        # if balance_factor < -1 and (angle, distance) > (node.left.angle, node.left.distance):
        #     return self._left_rotate(node)

        # if balance_factor < -1 and (angle, distance) < (node.left.angle, node.left.distance):
        #     node.right = self._right_rotate(node.right)
        #     return self._left_rotate(node)

        return node

    def delete(self, key):
        self.root = self._delete_helper(self.root, key)

    def _delete_helper(self, node: Node, key):
        if not node:
            return node
        elif key < node.key:
            node.left = self._delete_helper(node.left, key)
        elif key > node.key:
            node.right = self._delete_helper(node.right, key)
        else:
            if not node.left and not node.right:
                node = None
            elif not node.left:
                node = node.right
            elif not node.right:
                node = node.left
            else:
                min_node = self._find_min(node.right)
                node.key = min_node.key
                node.right = self._delete_helper(node.right, min_node.key)

        if not node:
            return node

        node.height = 1 + max(self._get_height(node.left), self._get_height(node.right))
        balance = self._get_balance_factor(node)

        if balance > 1 and self._get_balance_factor(node.left) >= 0:
            return self._right_rotate(node)
        if balance < -1 and self._get_balance_factor(node.right) <= 0:
            return self._left_rotate(node)
        if balance > 1 and self._get_balance_factor(node.left) < 0:
            node.left = self._left_rotate(node.left)
            return self._right_rotate(node)
        if balance < -1 and self._get_balance_factor(node.right) > 0:
            node.right = self._right_rotate(node.right)
            return self._left_rotate(node)

        return node

    def _get_height(self, current_node):
        if current_node is None:
            return 0
        return current_node.height

    def _get_balance_factor(self, current_node):
        if current_node is None:
            return 0
        return self._get_height(current_node.left) - self._get_height(current_node.right)

    def _left_rotate(self, z):
        y = z.right
        T2 = y.left

        y.left = z
        z.right = T2

        z.height = 1 + max(self._get_height(z.left), self._get_height(z.right))
        y.height = 1 + max(self._get_height(y.left), self._get_height(y.right))

        return y

    def _right_rotate(self, z):
        y = z.left
        T3 = y.right

        y.right = z
        z.left = T3

        z.height = 1 + max(self._get_height(z.left), self._get_height(z.right))
        y.height = 1 + max(self._get_height(y.left), self._get_height(y.right))

        return y

    def _find_min(self, root: Node):
        if root is None or root.left is None:
            return root
        return self._find_min(root.left)

    def find_min(self):
        return self._find_min(self.root)

    def pretty_print(self):
        self._print_helper(self.root, "", True)

    def _print_helper(self, current_node, prefix, is_left):
        if current_node is not None:
            self._print_helper(current_node.right, prefix + ("│   " if is_left else "    "), False)
            print(prefix + ("└── " if is_left else "┌── ") + str(current_node.key))
            self._print_helper(current_node.left, prefix + ("    " if is_left else "│   "), True)


class BinarySearchTree:
    def __init__(self):
        self.root = None

    def insert(self, value, edge):
        self.root = self._insert_helper(self.root, value, edge)

    def _insert_helper(self, current_node, value, edge):
        if current_node is None:
            return Node(value, edge)

        if value < current_node.value:
            current_node.left = self._insert_helper(current_node.left, value, edge)
        else:
            current_node.right = self._insert_helper(current_node.right, value, edge)

        return current_node

    def pretty_print(self):
        self._print_helper(self.root, "", True)

    def _print_helper(self, current_node, prefix, is_left):
        if current_node is not None:
            self._print_helper(current_node.right, prefix + ("│   " if is_left else "    "), False)
            print(prefix + ("└── " if is_left else "┌── ") + str(current_node.value))
            self._print_helper(current_node.left, prefix + ("    " if is_left else "│   "), True)


if __name__ == "__main__":
    # create a new balanced binary search tree
    bst = BalancedBinarySearchTree()
    print("Min: ", bst.find_min())
    # insert some values into the tree
    bst.insert((30, 10), "test5")
    bst.insert((10, 20), "test1")
    bst.insert((50, 20), "test2")
    bst.insert((10, 10), "test3")
    bst.insert((40, 20), "test4")
    bst.insert((40, 20), "test4")
    bst.insert((40, 20), "test4")
    bst.insert((40, 20), "test4")
    bst.insert((40, 10), "test4")
    bst.insert((30, 20), "test5")
    bst.insert((20, 20), "test6")
    bst.insert((25, 20), "test7")
    # print out the values in sorted order
    def inorder_traversal(current_node):
        if current_node:
            inorder_traversal(current_node.left)
            print(current_node)
            inorder_traversal(current_node.right)

    inorder_traversal(bst.root)
    print(bst.pretty_print())
    print("min node: ", bst.find_min())
    bst.delete((40, 10))
    bst.delete((10, 10))
    print(bst.pretty_print())
    print("min node: ", bst.find_min())
    inorder_traversal(bst.root)
    bst.delete((10, 20))
    print("min node: ", bst.find_min())
    inorder_traversal(bst.root)
    bst.insert((10, 10), "test00")
    print("min node: ", bst.find_min())
