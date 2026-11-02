# --- build stage ---
FROM node:22-alpine AS builder
WORKDIR /app
COPY tools/mission_control/frontend/package*.json ./
RUN npm ci
COPY tools/mission_control/frontend/ .
RUN npm run build

# --- serve stage ---
FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
# Rendered to /etc/nginx/conf.d/default.conf at start, with the backend's
# address from mc-backend.envsh. Only MC_BACKEND is substituted: nginx's own
# $variables in the config are left alone.
ENV NGINX_ENVSUBST_FILTER=MC_BACKEND
COPY docker/mission_control_frontend/nginx.conf.template /etc/nginx/templates/default.conf.template
COPY --chmod=755 docker/mission_control_frontend/mc-backend.envsh /docker-entrypoint.d/19-mc-backend.envsh
EXPOSE 80
