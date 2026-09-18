#!/usr/bin/env python3
"""Decode the two 9600/8N1 desk UART channels from a sigrok .sr capture."""

from __future__ import annotations

import argparse
import zipfile
from dataclasses import dataclass


@dataclass(frozen=True)
class Byte:
    sample: int
    value: int


def decode_uart(samples: bytes, channel: int, sample_rate: int, baud: int) -> list[Byte]:
    mask = 1 << channel
    bit_samples = sample_rate / baud
    decoded: list[Byte] = []
    index = 1
    while index + int(10 * bit_samples) < len(samples):
        if samples[index - 1] & mask and not samples[index] & mask:
            value = 0
            for bit in range(8):
                point = round(index + (1.5 + bit) * bit_samples)
                if samples[point] & mask:
                    value |= 1 << bit
            stop_point = round(index + 9.5 * bit_samples)
            if samples[stop_point] & mask:
                decoded.append(Byte(index, value))
            index += round(9.5 * bit_samples)
        else:
            index += 1
    return decoded


def checksum(frame: list[int]) -> int:
    return (sum(frame[2:6]) - 1) & 0xFF


def frames(decoded: list[Byte], header: tuple[int, int], sample_rate: int, baud: int):
    max_gap = round(12 * sample_rate / baud)
    index = 0
    while index + 7 < len(decoded):
        candidate = decoded[index : index + 8]
        values = [item.value for item in candidate]
        contiguous = all(
            candidate[pos + 1].sample - candidate[pos].sample <= max_gap
            for pos in range(7)
        )
        if (
            values[:2] == list(header)
            and values[2] == 8
            and values[7] == checksum(values)
            and contiguous
        ):
            yield candidate[0].sample, values
            index += 8
        else:
            index += 1


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("capture")
    parser.add_argument("--sample-rate", type=int, default=1_000_000)
    parser.add_argument("--baud", type=int, default=9600)
    args = parser.parse_args()

    with zipfile.ZipFile(args.capture) as archive:
        samples = archive.read("logic-1-1")

    for channel, header, label in (
        (0, (0xAA, 0x55), "panel"),
        (1, (0x55, 0xAA), "controller"),
    ):
        print(f"[{label}]")
        decoded = decode_uart(samples, channel, args.sample_rate, args.baud)
        for sample, frame in frames(decoded, header, args.sample_rate, args.baud):
            seconds = sample / args.sample_rate
            data = " ".join(f"{value:02X}" for value in frame)
            print(f"{seconds:9.6f}  {data}")


if __name__ == "__main__":
    main()
