-- Export the actual commands of each compiled target into its own output directory.
rule("libht.compile_commands")
    after_load(function (target)
        if target:is_phony() or target:is_headeronly() then return end
        import("core.project.config")
        if not target:get("targetdir") then
            target:set("targetdir", path.join(config.builddir(), target:name(), config.mode()))
        end
        target:add("cleanfiles", path.join(target:targetdir(), "compile_commands.json"))
    end)
    after_build(function (target)
        if target:is_phony() or target:is_headeronly() then return end
        import("core.tool.compiler")
        import("core.base.json")
        import("private.utils.executable_path")
        local entries = {}
        for _, batch in pairs(target:sourcebatches()) do
            if batch.sourcekind and batch.objectfiles then
                for index, source in ipairs(batch.sourcefiles) do
                    local arguments = table.join(compiler.compargv(source, batch.objectfiles[index],
                        {target = target, sourcekind = batch.sourcekind, rawargs = true}))
                    arguments[1] = executable_path(arguments[1])
                    table.insert(entries, {directory = os.projectdir(), file = path.absolute(source),
                        arguments = arguments})
                end
            end
        end
        table.sort(entries, function (a, b) return a.file < b.file end)
        local filename = path.join(target:targetdir(), "compile_commands.json")
        local content = json.encode(entries, {pretty = true}) .. "\n"
        if not os.isfile(filename) or io.readfile(filename) ~= content then
            os.mkdir(target:targetdir())
            io.writefile(filename, content)
        end
    end)
rule_end()
