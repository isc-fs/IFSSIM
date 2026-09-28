# bench-view, served by gunicorn. Build context: tools/sim_benchmark/tracking.
FROM python:3.12-slim
# git: the Launch page pins branches and PRs to commits (git ls-remote) in the mounted clone,
# which belongs to another user on the host
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates \
    && rm -rf /var/lib/apt/lists/* && git config --system --add safe.directory '*'
COPY --from=ghcr.io/astral-sh/uv:0.12 /uv /bin/uv
WORKDIR /app
ENV UV_COMPILE_BYTECODE=1 UV_LINK_MODE=copy
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --extra viewer --no-install-project --no-dev
COPY bench_tracking ./bench_tracking
RUN uv sync --frozen --extra viewer --no-dev && uv pip install "gunicorn>=23,<24"
ENV PATH=/app/.venv/bin:$PATH
EXPOSE 8050
# one process (the run cache lives in it), several threads
CMD ["gunicorn", "--bind", "0.0.0.0:8050", "--workers", "1", "--threads", "8", \
     "--timeout", "180", "bench_tracking.viewer.app:wsgi()"]
