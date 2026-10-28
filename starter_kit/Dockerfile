# A minimal generator container. Copy this directory, replace generate.py, and submit the image.
#
# The interface is three lines long and nothing further is required:
#   read  F11_SEED and F11_N from the environment
#   write /out/submission.json
#   exit  0
#
# The container runs with no network, a read-only root filesystem and dropped capabilities, so it
# cannot fetch scenarios at evaluation time or write anywhere but /out. It is run twice on the same
# seed and the two outputs must be byte-identical: a generator that is not reproducible cannot be
# audited after the fact.
FROM python:3.11.18-slim@sha256:ffe4b0c76963e64f05484d74dbb3f67fad1effcf3c5c22c2235f0fafc8a9c98f

# numpy is present because most generators use it. Further dependencies may be added freely.
RUN pip install --no-cache-dir numpy==2.1.3 && \
    useradd -m -u 10001 entrant

WORKDIR /app
COPY generate.py /app/generate.py
USER entrant

ENTRYPOINT ["python", "/app/generate.py"]
