from .symbol import Symbol


class BranchOpen(Symbol):
    def __str__(self) -> str:
        return "BranchOpen"


class BranchClose(Symbol):
    def __str__(self) -> str:
        return "BranchClose"
