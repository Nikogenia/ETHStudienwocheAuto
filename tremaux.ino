#include "constants.h"

bool areJunctionsEqual(Junction leftJunction, Junction rightJunction)
{
    return leftJunction.left == rightJunction.left &&
           leftJunction.forward == rightJunction.forward &&
           leftJunction.right == rightJunction.right;
}

bool pushTremauxNode(Junction junction)
{
    if (tremauxStackPointer + 1 >= TREMAUX_STACK_SIZE)
    {
        return false;
    }

    tremauxStackPointer++;
    tremauxStack[tremauxStackPointer].junction = junction;
    tremauxStack[tremauxStackPointer].nextBranchIndex = TREMAUX_BRANCH_LEFT;
    return true;
}

void popTremauxNode()
{
    if (tremauxStackPointer >= 0)
    {
        tremauxStackPointer--;
    }
}

byte selectTremauxBranch(TremauxNode &node)
{
    for (byte branchIndex = node.nextBranchIndex; branchIndex <= TREMAUX_BRANCH_RIGHT; branchIndex++)
    {
        if (branchIndex == TREMAUX_BRANCH_LEFT && node.junction.left)
        {
            node.nextBranchIndex = TREMAUX_BRANCH_FORWARD;
            return TREMAUX_BRANCH_LEFT;
        }

        if (branchIndex == TREMAUX_BRANCH_FORWARD && node.junction.forward)
        {
            node.nextBranchIndex = TREMAUX_BRANCH_RIGHT;
            return TREMAUX_BRANCH_FORWARD;
        }

        if (branchIndex == TREMAUX_BRANCH_RIGHT && node.junction.right)
        {
            node.nextBranchIndex = TREMAUX_BRANCH_UTURN;
            return TREMAUX_BRANCH_RIGHT;
        }
    }

    node.nextBranchIndex = TREMAUX_BRANCH_UTURN;
    return TREMAUX_BRANCH_UTURN;
}

void applyTremauxBranch(byte branch)
{
    if (branch == TREMAUX_BRANCH_LEFT)
    {
        state = SEARCH_LINE_LEFT;
        searchLineStart = millis();
    }
    else if (branch == TREMAUX_BRANCH_FORWARD)
    {
        state = FOLLOW_LINE;
    }
    else if (branch == TREMAUX_BRANCH_RIGHT)
    {
        state = SEARCH_LINE_RIGHT;
        searchLineStart = millis();
    }
    else
    {
        popTremauxNode();
        state = SEARCH_LINE_LEFT;
        searchLineStart = millis();
    }
}

void decideIntersectionTremaux()
{
    if (tremauxStackPointer < 0 ||
        !areJunctionsEqual(
            tremauxStack[tremauxStackPointer].junction,
            currentJunction))
    {
        if (!pushTremauxNode(currentJunction))
        {
            decideIntersectionLefthand();
            return;
        }
    }

    TremauxNode &currentNode = tremauxStack[tremauxStackPointer];
    byte selectedBranch = selectTremauxBranch(currentNode);
    applyTremauxBranch(selectedBranch);

    currentJunction = {false, false, false};
    intersectionStart = 0;
}
