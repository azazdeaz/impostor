# pyre-strict
from dataclasses import dataclass
import typing
import serde_types as st
import bincode

@dataclass(frozen=True)
class Plant:
    stems: typing.Sequence["Stem"]

    def bincode_serialize(self) -> bytes:
        return bincode.serialize(self, Plant)

    @staticmethod
    def bincode_deserialize(input: bytes) -> 'Plant':
        v, buffer = bincode.deserialize(input, Plant)
        if buffer:
            raise st.DeserializationError("Some input bytes were not read");
        return v


@dataclass(frozen=True)
class Pose:
    position: typing.Tuple[st.float32, st.float32, st.float32]
    orientation: typing.Tuple[st.float32, st.float32, st.float32, st.float32]

    def bincode_serialize(self) -> bytes:
        return bincode.serialize(self, Pose)

    @staticmethod
    def bincode_deserialize(input: bytes) -> 'Pose':
        v, buffer = bincode.deserialize(input, Pose)
        if buffer:
            raise st.DeserializationError("Some input bytes were not read");
        return v


@dataclass(frozen=True)
class Stem:
    rings: typing.Sequence["StemRing"]

    def bincode_serialize(self) -> bytes:
        return bincode.serialize(self, Stem)

    @staticmethod
    def bincode_deserialize(input: bytes) -> 'Stem':
        v, buffer = bincode.deserialize(input, Stem)
        if buffer:
            raise st.DeserializationError("Some input bytes were not read");
        return v


@dataclass(frozen=True)
class StemRing:
    pose: "Pose"
    radius: st.float32

    def bincode_serialize(self) -> bytes:
        return bincode.serialize(self, StemRing)

    @staticmethod
    def bincode_deserialize(input: bytes) -> 'StemRing':
        v, buffer = bincode.deserialize(input, StemRing)
        if buffer:
            raise st.DeserializationError("Some input bytes were not read");
        return v

