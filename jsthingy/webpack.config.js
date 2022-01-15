"use strict";

const path = require('path');
const CopyWebpackPlugin = require("copy-webpack-plugin");
const { CleanWebpackPlugin } = require("clean-webpack-plugin");

module.exports = [
    {
        context: path.join(__dirname, "src"),
        entry: {
            module: "./tramroute.ts",
        },
        devtool: "source-map",
        output: {
            filename: "tramroute.js",
            path: path.join(__dirname, "dist"),
        },
        plugins: [
            new CleanWebpackPlugin({
                cleanOnceBeforeBuildPatterns: [
                    "dist",
                ],
            }),
            new CopyWebpackPlugin({
                patterns: [
                    "*.ts", "*.css", "*.html",
                    "../node_modules/leaflet/dist/leaflet.css",
                    {
                        context: "../node_modules/leaflet/dist/images/",
                        from: "*",
                        to: "images/",
                    },
                ],
            }),
        ],
        resolve: {
            extensions: [".ts", ".js"],
        },
        module: {
            rules: [
                {
                    test: /\.ts$/,
                    use: [
                        {
                            loader: "ts-loader",
                        },
                    ],
                    exclude: /(node_modules)/,
                },
                {
                    test: /\.css$/,
                    use: [
                        {
                            loader: "style-loader",
                        },
                        {
                            loader: "css-loader",
                            options: {
                                importLoaders: 1,
                                sourceMap: true,
                            },
                        },
                    ],
                },
            ],
        },
    },
];
