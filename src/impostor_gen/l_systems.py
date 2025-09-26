from collections.abc import Sequence
from typing import List, Tuple
from pydantic import BaseModel
import rerun as rr

from impostor_gen.branch_symbols import BranchOpen
from impostor_gen.symbol import Symbol
from impostor_gen.branch_symbols import BranchClose
from impostor_gen.context import Context
from impostor_gen.rule import Rule, Writer




class LSystem(BaseModel):
    world: Sequence[Symbol]
    rules: Sequence[Rule]

    class Config:
        arbitrary_types_allowed = True

    def iterate(self, n: int = 1):
        context = Context()
        for _ in range(n):
            pointer = 0
            new_world: List[Symbol] = []
            while pointer < len(self.world):
                context.feed_node(self.world[pointer])
                for rule in self.rules:
                    writer = Writer(world=self.world, pointer=pointer)
                    rule.apply(writer, context)
                    if writer.replacement is not None:
                        new_world.extend(writer.replacement)
                        pointer += writer.window
                        break
                else:
                    new_world.append(self.world[pointer])
                    pointer += 1
            self.world = new_world    

    def log_graph(self):
        node_ids = [str(i) for i in range(len(self.world))]
        node_labels = [str(symbol) for symbol in self.world]
        edges: List[Tuple[str, str]] = []

        branch_stack: List[int] = []

        curr_id = 0
        for next_id, next_symbol in enumerate(self.world[1:], start=1):
            if isinstance(next_symbol, BranchOpen):
                branch_stack.append(curr_id)

            edges.append((node_ids[curr_id], node_ids[next_id]))
            curr_id = next_id

            if isinstance(next_symbol, BranchClose):
                if branch_stack:
                    curr_id = branch_stack.pop()

        rr.log(
            "world_graph",
            rr.GraphNodes(node_ids=node_ids, labels=node_labels),
            rr.GraphEdges(
                edges=edges,
                graph_type="directed",
            ),
        )

    def log_as_markdown(self):
        tab_size = 0
        markdown_lines: List[str] = []
        for symbol in self.world:
            if isinstance(symbol, BranchClose):
                tab_size = max(0, tab_size - 1)

            indent = "  " * tab_size
            markdown_lines.append(f"{indent}- {str(symbol)}")

            if isinstance(symbol, BranchOpen):
                tab_size += 1

        rr.log(
            "markdown",
            rr.TextDocument(
                "\n".join(markdown_lines),
                media_type=rr.MediaType.MARKDOWN,
            ),
        )
